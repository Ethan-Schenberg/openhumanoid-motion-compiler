"""Local-only FastAPI dashboard for staged OHMC training runs."""

from __future__ import annotations

import asyncio
import json
import os
import signal
import subprocess
import threading
from html import escape
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Form, HTTPException
from fastapi.responses import (
    FileResponse,
    HTMLResponse,
    JSONResponse,
    RedirectResponse,
    StreamingResponse,
)

from .errors import OhmcError
from .training import (
    TrainingStore,
    execute_run,
    load_training_recipe,
    prepare_run,
)

STATE_LABELS = {
    "created": "已创建",
    "preflight": "环境预检",
    "training": "训练中",
    "evaluating": "等待评估",
    "sim2sim": "跨仿真验证",
    "awaiting_hardware_review": "等待人工审核",
    "hardware_candidate": "真机候选",
    "failed": "失败",
    "cancelled": "已取消",
}


def _layout(title: str, body: str) -> str:
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>{escape(title)} · OHMC</title>
  <script src="https://unpkg.com/htmx.org@2.0.8" defer></script>
  <style>
    :root {{ color-scheme: dark; --bg:#0b1020; --card:#151c31; --line:#2b3657; --blue:#65a7ff; --green:#53d49b; --red:#ff7777; --muted:#aab5d1; }}
    * {{ box-sizing:border-box }} body {{ margin:0; font:15px/1.55 system-ui,sans-serif; background:var(--bg); color:#f4f7ff }}
    header,main {{ max-width:1120px; margin:auto; padding:22px }} header {{ display:flex; justify-content:space-between; align-items:center }}
    a {{ color:var(--blue) }} .grid {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(290px,1fr)); gap:16px }}
    .card {{ background:var(--card); border:1px solid var(--line); border-radius:14px; padding:18px }}
    .safe {{ border-left:4px solid var(--green) }} .warning {{ border-left:4px solid #ffbd59 }}
    .status {{ display:inline-block; padding:3px 9px; border-radius:99px; background:#263453 }}
    input,button {{ width:100%; margin-top:8px; padding:11px; border:1px solid var(--line); border-radius:8px; color:inherit; background:#0e1528 }}
    button {{ cursor:pointer; background:#225ca8; font-weight:650 }} button.secondary {{ background:#263453 }} button.danger {{ background:#873b49 }}
    table {{ width:100%; border-collapse:collapse }} td,th {{ padding:9px; border-bottom:1px solid var(--line); text-align:left }}
    code,pre {{ background:#0a0f1d; border-radius:8px }} pre {{ padding:13px; white-space:pre-wrap; overflow:auto }}
    .muted {{ color:var(--muted) }} .ok {{ color:var(--green) }} .bad {{ color:var(--red) }}
  </style>
</head>
<body><header><h2><a href="/" style="color:inherit;text-decoration:none">OHMC 2.0</a></h2><span class="status">硬件传输：关闭</span></header><main>{body}</main></body></html>"""


class DashboardJobs:
    """Owns only processes started by this dashboard instance."""

    def __init__(self, store: TrainingStore) -> None:
        self.store = store
        self._lock = threading.Lock()
        self._processes: dict[str, subprocess.Popen[Any]] = {}

    def _remember(self, run_id: str, process: subprocess.Popen[Any]) -> None:
        with self._lock:
            self._processes[run_id] = process

    def start(self, run_id: str) -> None:
        manifest = self.store.get(run_id)
        if manifest["state"] != "preflight" or manifest["execution"]["status"] not in {
            "ready",
            "queued",
        }:
            raise OhmcError("run is not ready to start")
        self.store.set_execution(run_id, "queued", message="queued by local dashboard")

        def target() -> None:
            try:
                execute_run(
                    self.store,
                    run_id,
                    process_started=lambda process: self._remember(run_id, process),
                )
            finally:
                with self._lock:
                    self._processes.pop(run_id, None)

        threading.Thread(target=target, name=f"ohmc-{run_id}", daemon=True).start()

    def recover(self, run_id: str) -> None:
        manifest = self.store.get(run_id)
        if (
            manifest["state"] != "training"
            or manifest["execution"]["status"] != "blocked"
        ):
            raise OhmcError("run is not an interrupted recoverable training run")
        self.store.set_execution(
            run_id, "queued", message="recovery queued by local dashboard"
        )

        def target() -> None:
            try:
                # execute_run accepts blocked; restore that marker immediately before launch.
                self.store.set_execution(
                    run_id,
                    "blocked",
                    message="recovery worker acquired the interrupted run",
                )
                execute_run(
                    self.store,
                    run_id,
                    resume=True,
                    process_started=lambda process: self._remember(run_id, process),
                )
            finally:
                with self._lock:
                    self._processes.pop(run_id, None)

        threading.Thread(
            target=target, name=f"ohmc-recover-{run_id}", daemon=True
        ).start()

    def _process(self, run_id: str) -> subprocess.Popen[Any]:
        with self._lock:
            process = self._processes.get(run_id)
        if process is None or process.poll() is not None:
            raise OhmcError(
                "this dashboard instance does not own a running process for the run"
            )
        return process

    def pause(self, run_id: str) -> None:
        process = self._process(run_id)
        if not hasattr(signal, "SIGSTOP"):
            raise OhmcError("pause is supported only on the WSL/Linux training host")
        os.killpg(process.pid, signal.SIGSTOP)
        self.store.set_execution(
            run_id, "paused", pid=process.pid, message="training paused"
        )

    def resume(self, run_id: str) -> None:
        process = self._process(run_id)
        if not hasattr(signal, "SIGCONT"):
            raise OhmcError("resume is supported only on the WSL/Linux training host")
        os.killpg(process.pid, signal.SIGCONT)
        self.store.set_execution(
            run_id, "running", pid=process.pid, message="training resumed"
        )

    def cancel(self, run_id: str) -> None:
        process = self._process(run_id)
        self.store.set_execution(
            run_id, "cancelled", message="cancel requested by operator"
        )
        self.store.transition(
            run_id, "cancelled", "operator cancelled the training stage"
        )
        os.killpg(process.pid, signal.SIGTERM)


def _run_table(runs: list[dict[str, Any]]) -> str:
    if not runs:
        return '<p class="muted">还没有训练任务。</p>'
    rows = []
    for run in runs:
        rows.append(
            "<tr>"
            f'<td><a href="/runs/{escape(run["run_id"])}">{escape(run["run_id"])}</a></td>'
            f"<td>{escape(run['recipe']['id'])}</td>"
            f"<td>{escape(STATE_LABELS.get(run['state'], run['state']))}</td>"
            f"<td>{escape(run['execution']['status'])}</td>"
            "</tr>"
        )
    return (
        "<table><thead><tr><th>任务</th><th>配方</th><th>阶段</th><th>执行</th></tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>"
    )


def _flatten_metrics(
    value: dict[str, Any], prefix: str = ""
) -> list[tuple[str, float]]:
    flattened: list[tuple[str, float]] = []
    for key, item in value.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(item, dict):
            flattened.extend(_flatten_metrics(item, name))
        elif isinstance(item, (int, float)) and not isinstance(item, bool):
            flattened.append((name, float(item)))
    return flattened


def _metrics_panel(metrics: dict[str, Any]) -> str:
    values = _flatten_metrics(metrics.get("metrics", {}))
    if not values:
        return ""
    rows = "".join(
        f"<tr><td>{escape(name)}</td><td>{value:.6g}</td></tr>"
        for name, value in values
    )
    chart = ""
    try:
        import plotly.graph_objects as go
        from plotly.io import to_html

        figure = go.Figure(
            data=[
                go.Bar(x=[name for name, _ in values], y=[value for _, value in values])
            ]
        )
        figure.update_layout(
            title="评估指标",
            height=420,
            margin={"l": 45, "r": 20, "t": 55, "b": 150},
            paper_bgcolor="#151c31",
            plot_bgcolor="#151c31",
            font_color="#f4f7ff",
            xaxis_tickangle=-45,
        )
        chart = to_html(
            figure,
            full_html=False,
            include_plotlyjs=True,
            config={"displaylogo": False, "responsive": True},
        )
    except ImportError:
        chart = (
            '<p class="muted">安装 `.[web]` 后显示 Plotly 图表；数值表仍可审计。</p>'
        )
    return f'<section class="card" style="margin-top:16px"><h3>指标</h3>{chart}<table><tbody>{rows}</tbody></table></section>'


def _doctor_panel(run_dir: Path) -> str:
    path = run_dir / "doctor.json"
    if not path.is_file():
        return ""
    try:
        report = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return ""
    rows = []
    for check in report.get("checks", []):
        status_class = (
            "ok"
            if check.get("status") == "pass"
            else "bad"
            if check.get("status") == "fail"
            else "muted"
        )
        fix = check.get("fix", "")
        rows.append(
            f'<tr><td class="{status_class}">{escape(str(check.get("status", "")))}</td>'
            f"<td>{escape(str(check.get('name', '')))}</td>"
            f"<td>{escape(str(check.get('detail', '')))}</td>"
            f"<td>{escape(str(fix))}</td></tr>"
        )
    return (
        '<section class="card" style="margin-top:16px"><h3>环境诊断</h3><table><thead><tr><th>结果</th><th>检查项</th><th>详情</th><th>怎么处理</th></tr></thead><tbody>'
        + "".join(rows)
        + "</tbody></table></section>"
    )


def create_app(
    *,
    runs_dir: Path,
    default_recipe: Path,
    robot_profile: Path,
    project_root: Path,
) -> FastAPI:
    run_schema = project_root / "schemas" / "run-manifest-v0.1.schema.json"
    recipe_schema = project_root / "schemas" / "training-recipe-v0.1.schema.json"
    profile_schema = project_root / "schemas" / "robot-profile-v0.1.schema.json"
    store = TrainingStore(runs_dir, run_schema_path=run_schema)
    store.recover_orphaned_runs()
    jobs = DashboardJobs(store)
    app = FastAPI(title="OHMC 2.0", version="0.1.0", docs_url="/api/docs")
    app.state.store = store
    app.state.jobs = jobs

    @app.exception_handler(OhmcError)
    async def ohmc_error(_request: Any, exc: OhmcError) -> JSONResponse:
        return JSONResponse(status_code=400, content={"error": str(exc)})

    @app.get("/", response_class=HTMLResponse)
    def home() -> str:
        body = f"""
<section class="grid">
  <article class="card safe"><h3>自主步态训练</h3><p>从零训练 X2 RGB-D 步态。一次只启动一个安全阶段，训练结束后仍需评估和人工审核。</p>
    <form method="post" action="/runs">
      <label>训练配方</label><input name="recipe_path" value="{escape(str(default_recipe.expanduser().resolve()))}">
      <button name="execute" value="true">创建、预检并启动</button>
      <button class="secondary" name="execute" value="false">只创建并预检</button>
    </form>
  </article>
  <article class="card warning"><h3>LinkCraft 快速表演</h3><p>用于视频/BVH动作资源。它不是强化学习结果，也不属于 OHMC 真机验证证据。</p>
    <a href="https://linkcraft.agibot.com/" target="_blank" rel="noreferrer"><button class="secondary">打开官方 LinkCraft</button></a>
  </article>
</section>
<section class="card" style="margin-top:16px"><h3>训练任务</h3>{_run_table(store.list())}</section>
"""
        return _layout("控制台", body)

    @app.post("/runs")
    def create_run_form(
        recipe_path: str = Form(...), execute: str = Form("false")
    ) -> RedirectResponse:
        path = Path(recipe_path).expanduser().resolve()
        recipe, issues = load_training_recipe(path, recipe_schema)
        if issues:
            raise OhmcError("invalid training recipe: " + "; ".join(issues))
        if recipe["robot_profile"] != robot_profile.stem:
            raise OhmcError(
                "recipe robot profile does not match the configured dashboard profile"
            )
        manifest, report = prepare_run(
            store,
            recipe,
            path,
            profile_path=robot_profile,
            profile_schema_path=profile_schema,
        )
        if execute == "true" and report["ready"]:
            jobs.start(manifest["run_id"])
        return RedirectResponse(f"/runs/{manifest['run_id']}", status_code=303)

    @app.get("/runs/{run_id}", response_class=HTMLResponse)
    def run_detail(run_id: str) -> str:
        manifest = store.get(run_id)
        events = store.events(run_id)
        run_dir = store.run_dir(run_id)
        log_path = run_dir / "training.log"
        log = ""
        if log_path.is_file():
            log = log_path.read_text(encoding="utf-8", errors="replace")[-12000:]
        controls = []
        if (
            manifest["state"] == "preflight"
            and manifest["execution"]["status"] == "ready"
        ):
            controls.append(
                f'<form method="post" action="/runs/{run_id}/start"><button>启动训练</button></form>'
            )
        if (
            manifest["state"] == "training"
            and manifest["execution"]["status"] == "blocked"
        ):
            controls.append(
                f'<form method="post" action="/runs/{run_id}/recover"><button>从课程检查点恢复</button></form>'
            )
        if manifest["execution"]["status"] == "running":
            controls.append(
                f'<form method="post" action="/runs/{run_id}/pause"><button class="secondary">暂停</button></form>'
            )
        if manifest["execution"]["status"] == "paused":
            controls.append(
                f'<form method="post" action="/runs/{run_id}/resume"><button>继续</button></form>'
            )
        if manifest["execution"]["status"] in {"running", "paused"}:
            controls.append(
                f'<form method="post" action="/runs/{run_id}/cancel"><button class="danger">取消当前阶段</button></form>'
            )
        event_rows = "".join(
            f"<tr><td>{escape(item['at'])}</td><td>{escape(item['kind'])}</td><td>{escape(item['message'])}</td></tr>"
            for item in events[-50:]
        )
        artifact_rows = []
        videos = []
        for role, artifact in sorted(manifest["artifacts"].items()):
            relative = artifact["path"]
            url = f"/api/runs/{run_id}/artifacts/{relative}"
            artifact_rows.append(
                f'<tr><td>{escape(role)}</td><td><a href="{escape(url)}">{escape(relative)}</a></td>'
                f"<td><code>{escape(artifact['sha256'][:12])}…</code></td></tr>"
            )
            if Path(relative).suffix.lower() in {".mp4", ".webm"}:
                videos.append(
                    f'<h4>{escape(role)}</h4><video controls preload="metadata" style="width:100%;max-height:520px" src="{escape(url)}"></video>'
                )
        artifacts_panel = (
            '<section class="card" style="margin-top:16px"><h3>产物与证据</h3><table><thead><tr><th>用途</th><th>文件</th><th>SHA-256</th></tr></thead><tbody>'
            + "".join(artifact_rows)
            + "</tbody></table>"
            + "".join(videos)
            + "</section>"
            if artifact_rows
            else ""
        )
        metrics_panel = ""
        metrics_path = run_dir / "evidence" / "metrics.json"
        if metrics_path.is_file():
            try:
                metrics_panel = _metrics_panel(
                    json.loads(metrics_path.read_text(encoding="utf-8"))
                )
            except (json.JSONDecodeError, OSError):
                pass
        last_sequence = events[-1]["sequence"] if events else 0
        body = f"""
<section class="grid"><article class="card"><h3>{escape(run_id)}</h3>
  <p>阶段：<span class="status">{escape(STATE_LABELS.get(manifest["state"], manifest["state"]))}</span></p>
  <p>执行：{escape(manifest["execution"]["status"])}</p><p class="muted">真机权限：无</p>{"".join(controls)}
</article><article class="card"><h3>下一步</h3><p>{escape(_next_step(manifest))}</p></article></section>
<section class="card" style="margin-top:16px"><h3>事件</h3><table><tbody>{event_rows}</tbody></table></section>
{_doctor_panel(run_dir)}{metrics_panel}{artifacts_panel}
<section class="card" style="margin-top:16px"><h3>训练日志</h3><pre id="log">{escape(log or "尚无日志")}</pre></section>
<script>
const events = new EventSource('/api/runs/{escape(run_id)}/stream?after={last_sequence}');
events.onmessage = () => setTimeout(() => location.reload(), 300);
</script>"""
        return _layout(run_id, body)

    def redirect(run_id: str) -> RedirectResponse:
        return RedirectResponse(f"/runs/{run_id}", status_code=303)

    @app.post("/runs/{run_id}/start")
    def start(run_id: str) -> RedirectResponse:
        jobs.start(run_id)
        return redirect(run_id)

    @app.post("/runs/{run_id}/pause")
    def pause(run_id: str) -> RedirectResponse:
        jobs.pause(run_id)
        return redirect(run_id)

    @app.post("/runs/{run_id}/recover")
    def recover(run_id: str) -> RedirectResponse:
        jobs.recover(run_id)
        return redirect(run_id)

    @app.post("/runs/{run_id}/resume")
    def resume(run_id: str) -> RedirectResponse:
        jobs.resume(run_id)
        return redirect(run_id)

    @app.post("/runs/{run_id}/cancel")
    def cancel(run_id: str) -> RedirectResponse:
        jobs.cancel(run_id)
        return redirect(run_id)

    @app.get("/api/health")
    def health() -> dict[str, Any]:
        return {"status": "ok", "hardware_transport": False}

    @app.get("/api/runs")
    def api_runs() -> list[dict[str, Any]]:
        return store.list()

    @app.get("/api/runs/{run_id}")
    def api_run(run_id: str) -> dict[str, Any]:
        return store.get(run_id)

    @app.get("/api/runs/{run_id}/events")
    def api_events(run_id: str, after: int = 0) -> list[dict[str, Any]]:
        return store.events(run_id, after)

    @app.get("/api/runs/{run_id}/stream")
    async def stream(run_id: str, after: int = 0) -> StreamingResponse:
        store.get(run_id)

        async def generate() -> Any:
            cursor = max(0, after)
            for _ in range(60):
                for item in store.events(run_id, cursor):
                    cursor = item["sequence"]
                    yield f"id: {cursor}\ndata: {json.dumps(item, ensure_ascii=False)}\n\n"
                await asyncio.sleep(1)

        return StreamingResponse(generate(), media_type="text/event-stream")

    @app.get("/api/runs/{run_id}/artifacts/{artifact_path:path}")
    def artifact(run_id: str, artifact_path: str) -> FileResponse:
        run_dir = store.run_dir(run_id)
        requested = (run_dir / artifact_path).resolve()
        try:
            requested.relative_to(run_dir)
        except ValueError as exc:
            raise HTTPException(status_code=404) from exc
        if not requested.is_file():
            raise HTTPException(status_code=404)
        return FileResponse(requested)

    return app


def _next_step(manifest: dict[str, Any]) -> str:
    state = manifest["state"]
    execution = manifest["execution"]["status"]
    if state == "preflight" and execution == "blocked":
        return "打开 doctor.json，按修复建议补齐 WSL2、NVIDIA 或 Isaac Lab 环境。"
    if state == "preflight":
        return "环境已经通过预检，可以启动本阶段训练。"
    if state == "training":
        if execution == "blocked":
            return (
                "上次进程已中断；确认 WSL2 与 GPU 正常后，从最近完成的课程检查点恢复。"
            )
        return "等待训练完成；可暂停或继续，切勿关闭 WSL2。"
    if state == "evaluating":
        return "使用 ohmc evaluate 导入100回合评估和独立 MuJoCo 结果。"
    if state == "awaiting_hardware_review":
        return "只可准备 Policy Bundle；仍未取得真机执行权限。"
    if state == "failed":
        return "查看失败门禁和日志，修正后创建新训练任务，失败不会自动进入真机。"
    if state == "cancelled":
        return "当前阶段已取消；检查后从保存的检查点创建恢复任务。"
    return "当前状态不会自动向真机阶段晋级。"
