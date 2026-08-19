from __future__ import annotations

from pathlib import Path

from fastapi.testclient import TestClient
from test_training import PROFILE_PATH, ROOT, fixture_recipe

from ohmc.web import create_app


def test_dashboard_is_local_training_ui_without_hardware_endpoint(
    tmp_path: Path,
) -> None:
    recipe_path, _recipe = fixture_recipe(tmp_path)
    app = create_app(
        runs_dir=tmp_path / "runs",
        default_recipe=recipe_path,
        robot_profile=PROFILE_PATH,
        project_root=ROOT,
    )
    with TestClient(app) as client:
        home = client.get("/")
        assert home.status_code == 200
        assert "自主步态训练" in home.text
        assert "硬件传输：关闭" in home.text
        assert "LinkCraft" in home.text
        assert client.get("/api/health").json() == {
            "status": "ok",
            "hardware_transport": False,
        }

        created = client.post(
            "/runs",
            data={"recipe_path": str(recipe_path), "execute": "false"},
            follow_redirects=False,
        )
        assert created.status_code == 303
        run_id = created.headers["location"].rsplit("/", 1)[-1]
        manifest = client.get(f"/api/runs/{run_id}").json()
        assert manifest["state"] == "preflight"
        assert manifest["execution"]["status"] == "ready"
        assert manifest["authority"]["hardware_transport"] is False

        paths = {route.path for route in app.routes}
        assert not any("hardware" in path or "joint" in path for path in paths)


def test_artifact_endpoint_rejects_path_escape(tmp_path: Path) -> None:
    recipe_path, _recipe = fixture_recipe(tmp_path)
    app = create_app(
        runs_dir=tmp_path / "runs",
        default_recipe=recipe_path,
        robot_profile=PROFILE_PATH,
        project_root=ROOT,
    )
    with TestClient(app) as client:
        created = client.post(
            "/runs",
            data={"recipe_path": str(recipe_path), "execute": "false"},
            follow_redirects=False,
        )
        run_id = created.headers["location"].rsplit("/", 1)[-1]
        assert (
            client.get(
                f"/api/runs/{run_id}/artifacts/%2E%2E/%2E%2E/pyproject.toml"
            ).status_code
            == 404
        )
