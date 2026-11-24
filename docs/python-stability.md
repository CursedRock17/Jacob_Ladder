# Python stability

## Tool ownership and setup

Use mise for the developer toolchain. `mise.toml` pins standalone Ruff, ty and
uv binaries; `mise install` does not install ROS, Python application packages,
or synchronize the hardware environment. The existing project dependencies and
`uv.lock` remain unchanged for Jetson deployments.

```sh
mise install
mise run check
```

Ruff was already configured as the formatter/linter in `pyproject.toml`. ty was
also configured, including Python 3.10 and source roots for local packages.
This setup makes those tools directly available through mise, rather than
installing duplicate copies in the test environment. No Black, Pylint, Conda or
Pixi is needed for these checks.

uv manages the existing isolated test environment and deployment dependencies;
mise manages tool versions. uv is pinned for reproducibility, but the static
tasks use only Ruff and ty. Do not run `uv sync` on a Mac just to lint: the root
project includes Jetson-specific dependencies that are not a Mac development
environment.

## Style and lint policy

Use the Google Python style guide as human-review guidance, not as a claim of
complete automated compliance. Ruff is the only formatter. Keep its existing
format defaults; do not import a third-party gist wholesale or reformat the
whole repository as part of a tooling change.

The explicit lint baseline retains Ruff's E4/E7/E9 and F rules and adds a small
set of correctness checks: B006 (mutable defaults), B008 (calls in defaults),
B012 (control flow in finally), B016 (raising literals), and B904 (exception
chaining). Broader naming, docstring and import-order rules can be proposed
separately after measuring their effect.

The existing F841 exception applies only to launch files, whose optional node
builders are intentionally left unused. Keep suppressions narrow, with an
explanation. Do not disable whole rule families to hide a workspace baseline.
Use `ruff check --fix <path>` or `ruff format <path>` deliberately on owned code;
the check tasks never modify files or enable unsafe fixes.

## Check scopes

| Command | Scope and prerequisites |
| --- | --- |
| `mise run lint` | All of `src/jl_blocks`, including ROS adapters, launch and tests; imports are not executed. |
| `mise run format-check` | The same mission-block scope, read-only. |
| `mise run typecheck` | `src/jl_blocks/jl_blocks`, except `ros/`, `core/loader.py` and `flight.py`. No external packages required. |
| `mise run check` | All three static tasks above; safe on a ROS-free Mac. |
| `mise run test` / `make check` | Existing full ROS-free type, unit, deployment and mission-file checks; uv provisions Python 3.10, pytest and PyYAML in `.check-venv`. |
| `mise run lint-all` | Workspace lint diagnostics, including ROS Python; no ROS installation needed. |
| `mise run format-check-all` | Workspace formatting diagnostics, read-only. |
| `mise run typecheck-all` | Workspace diagnostics using the available Python environment; target dependencies are required for meaningful ROS/hardware coverage. |

The Mac-only type scope excludes the two modules that directly import PyYAML,
as well as ROS adapters. Their callers still participate in analysis, but missing
external types limit what can be inferred. The full check uses the real PyYAML
environment and includes these modules and tests. This is scoped enforcement,
not proof that the whole workspace is type-correct. Unresolved imports remain
errors: no global suppression or fake ROS stubs are added. Existing source roots
remain shared between editor, Mac and target checks.

Block parameter schemas are intentionally independent nested dataclasses, not
subclasses of one shared schema. `Block.Params` is an explicitly dynamic
`type[Any]` default factory; concrete blocks retain their own typed `Params`
classes. This matches the existing YAML parameter boundary without disabling
override diagnostics throughout the project.

Both tools exclude the existing upstream/submodule paths in `pyproject.toml`;
ignored build/install/log output is not part of the baseline. Do not modify
vendored code to make checks pass.

## Rollout and CI

CI installs the mise-pinned tools and runs `mise run check` followed by
`make check`. The latter retains the existing full mission-block checks,
unit/deployment tests and mission YAML validation. Neither job needs ROS.

Workspace baseline tasks are intentionally not required CI gates yet. At
initial inspection, the old Ruff default rules found 23 lint errors and the
formatter found 15 files needing formatting outside the enforced scope. The
ROS-free whole-workspace ty run reported 335 diagnostics, including missing
ROS, hardware, test and other third-party imports; that number is not a count
of confirmed code defects. These are a local snapshot, not permanent allowances.

Expand enforcement one first-party package at a time: inspect diagnostics,
separate unavailable dependencies from genuine errors, fix owned code in a
focused change, verify it in the appropriate environment, then add that package
to the required tasks. Keep the full target type pass for a ROS/Jetson
environment; installing ROS on developer Macs is not part of this workflow.

## Exceptions and runtime safety

Static checks do not replace runtime safety or flight testing. Preserve existing
mission failure/abort behavior. For new code, catch specific expected exceptions
at system boundaries, add actionable context and chain translated exceptions
with `raise ... from ...`. Avoid swallowing unexpected failures; test both the
failure and cleanup paths. Changes to the flight exception architecture need a
separate design and simulation review, not an automatic lint fix.

## Editor setup

In VS Code, use `charliermarsh.ruff` for lint/format and `astral-sh.ty` for type
checking; these are distinct extensions. Point tool overrides at the paths from
`mise which ruff` and `mise which ty` if the editor does not inherit mise's PATH.
Select the appropriate Python environment for dependency-aware analysis. The
editor may show unresolved ROS imports on macOS; the bounded static task is the
portable gate, not an instruction to install ROS or suppress those diagnostics.
