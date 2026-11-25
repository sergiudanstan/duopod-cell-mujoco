# Security Policy

This is an offline physics simulation: a Python package (`duopod_mj`) that builds a MuJoCo model
and steps it, plus command-line tools that write MP4/PNG/JSON files. It runs no server, opens no
network connection and handles no accounts or secrets. It is a sizing and logic check, not a
validated machine, and nothing here should be used to commission or certify a real robot cell.

## Supported versions

| Version | Supported |
|---|---|
| 0.1.x | Current |

Single maintainer; only the latest release receives fixes.

## Reporting a vulnerability

**Please do not open a public issue for a security problem.** Report it privately:
**[Report a vulnerability](https://github.com/sergiudanstan/duopod-cell-mujoco/security/advisories/new)**
(Security, Advisories, Report a vulnerability).

| Stage | Target |
|---|---|
| Acknowledgement | within 3 business days |
| Triage | within 10 business days |
| Fix, or a documented decision not to fix | within 90 days |

These are best-effort targets from a single unpaid maintainer, not a contractual SLA.
There is no bug bounty; credit is offered in the advisory and changelog.

## Scope

In scope: unsafe handling of the files the tools read or write. `tools/export_meshes.py` loads a local
vendor GLB with `trimesh` (installed ad hoc with `uv run --with trimesh`, so it is not covered by
`uv.lock`) and writes STL files; `duopod_mj/model.py` inserts the STL file names into the generated
MJCF XML text; `tools/export_trajectory.py` writes a JSON file to a path given on the command line,
by default into a sibling repository. Also in scope: anything in this repository's workflows that
could publish a release not built from `main`.

Out of scope: vulnerabilities in MuJoCo, numpy, imageio or trimesh themselves (report upstream), the
vendor CAD, which is not redistributed here, and the accuracy of the simulation. A wrong torque or
clearance result is a correctness bug; file it as a normal issue.

## Supply chain

CI runs the physics tests with dependencies installed from `uv.lock` (`uv sync --locked`), `pip-audit`
on the locked runtime dependencies and on the hash-pinned CI toolchain, CodeQL, and `zizmor` on the
workflows. Actions are pinned to full commit SHAs and kept current by Dependabot. Releases attach a
source archive and a CycloneDX SBOM of the locked runtime dependencies, both with Sigstore build provenance:

```
gh attestation verify duopod-cell-mujoco-v0.1.0.tar.gz --repo sergiudanstan/duopod-cell-mujoco
```
