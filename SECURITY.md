# Security Policy

## Supported versions

OHMC is pre-alpha. Security fixes are applied to the latest `main` branch until the first stable release defines a longer support policy.

## Reporting a vulnerability

Use GitHub's private vulnerability reporting feature for this repository. Do not open a public issue for vulnerabilities involving command injection, dependency acquisition, artifact verification, robot command encoding, authentication material, or a path that could cause unintended physical motion.

Include the affected revision, reproduction steps, expected impact, and whether physical hardware is involved. Do not test a report on a robot you do not own or have explicit permission to operate.

## Robot-safety boundary

Passing OHMC's offline validation or simulator replay does not certify a motion as safe for physical execution. Physical integrations must retain vendor safety systems, an operator-controlled stop path, platform-specific limits, command ownership checks, and explicit enablement.

