# Dev container

A VS Code dev container that opens this repository in a ready-to-run MuJoCo simulation
environment. It is for developers who want the full dependency set — Python, MuJoCo, the test
suite, the interactive viewer — without installing any of it on the host.

## Prerequisites

- Docker Desktop, with the WSL 2 backend
- VS Code with the Dev Containers extension
- VcXsrv — only if you want the interactive viewer, see [The interactive viewer](#the-interactive-viewer)

## Open the repository in the container

1. Start Docker Desktop.
2. Open this repository in VS Code, press `F1`, and run
   `Dev Containers: Rebuild and Reopen in Container`.
3. Wait for the image build and the install step. The install ends with a test run, so a green
   finish means the environment works.

## What is installed

| Component | Details |
| --- | --- |
| Python | 3.11, the version CI tests with |
| Package | `litegrip-mujoco`, editable with `.[dev]` extras |
| MuJoCo | latest 3.x, plus the Mesa software-GL, GLX, EGL, OSMesa and X11 client libraries |
| gh CLI, Node 22 | `gh` for GitHub operations, Node for the CI markdown lint |

The simulation needs no GPU. The gripper model is tiny, MuJoCo runs it faster than real time on
CPU, and the viewer renders with Mesa software GL.

pip is pointed at the Tsinghua TUNA mirror (`PIP_INDEX_URL` in the Dockerfile). Delete that line
if your network reaches PyPI directly.

## Everyday commands

The working directory is the repository root.

```bash
python -m pytest tests/ -v                                  # test suite
npx --yes markdownlint-cli2@0.23.3 "README.md" "README.zh-CN.md" "docs/*.md"   # the CI lint
python examples/01_hello_sim.py --no-render                # examples, headless
python examples/02_move_sim.py
python examples/03_trajectory.py
```

## The interactive viewer

The MuJoCo viewer window needs an X server on the Windows host. The container sets
`DISPLAY=host.docker.internal:0.0` for you, so install VcXsrv and the window appears:

1. Install VcXsrv from <https://sourceforge.net/projects/vcxsrv/>.
2. Run XLaunch with: Multiple windows, display number 0, and **Disable access control** checked.
3. Start VcXsrv, then run `python examples/01_hello_sim.py` — the viewer opens and animates.

Do not run an example with the viewer when no X server is running — `render=True` fails with a
GLFW error there. Use `--no-render`, or delete the `remoteEnv` block in `devcontainer.json` for
headless-only work.

A run that opened the viewer can end with `Segmentation fault` after the example prints its
completion line. That is the upstream GL teardown crash described in the main README, not a
simulation failure; `--no-render` is unaffected.

On a Linux host, delete the `remoteEnv` block and set `DISPLAY` to your own X server or Wayland
session instead.
