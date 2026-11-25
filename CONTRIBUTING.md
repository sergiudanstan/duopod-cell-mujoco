# Contributing

Contributions are welcome. This is a single-maintainer project, so please open an issue
before starting anything substantial.

**Security problems do not belong in issues.** Follow [SECURITY.md](SECURITY.md).

## Branches, pull requests and releases

- `main` is the only long-lived branch and is protected: changes land through a pull
  request with the required checks green, commits on `main` are signed, and force-pushes
  are refused. Pull requests are squash-merged; GitHub deletes the branch on merge.
- Branch names: `<type>/<topic>` (`feat/`, `fix/`, `docs/`, `chore/`, `release/`);
  agents use `agent/<name>/<topic>`.
- A release is a pull request that bumps `version` under `[project]` in `pyproject.toml` (and the
  matching entry in `uv.lock`, which `uv lock` rewrites) and moves `[Unreleased]` in
  [CHANGELOG.md](CHANGELOG.md) to a dated version. After it merges, a signed `vX.Y.Z` tag on that
  commit triggers `release.yml`, which refuses to run if the tag, `pyproject.toml` and the
  changelog disagree. Tags are never moved or deleted.

## Running it locally

```bash
uv sync --locked
uv run pytest -q -s                       # physics checks, about 15 s, headless
uv run pytest -q --perturb -k test_no_carton_collision_while_held   # must FAIL
uv run python tools/run_cycle.py          # headless run with a short report
```

Rendering (`tools/render.py`) and the interactive viewer need a display or an OpenGL backend and are
not run in CI. CI also runs `ruff check --select E9,F63,F7,F82` (syntax errors and undefined names only).
