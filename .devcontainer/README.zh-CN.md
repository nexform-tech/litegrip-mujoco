# Dev 容器

VS Code dev 容器，把本仓库放进一个开箱即用的 MuJoCo 仿真环境。给不想在宿主机上安装任何
依赖的开发者用：Python、MuJoCo、测试套件、交互式查看器都在容器里装好。

## 前置条件

- Docker Desktop，使用 WSL 2 后端
- 安装了 Dev Containers 扩展的 VS Code
- VcXsrv —— 只有想用交互式查看器时才需要，见[交互式查看器](#交互式查看器)

## 在容器中打开仓库

1. 启动 Docker Desktop。
2. 在 VS Code 中打开本仓库，按 `F1`，执行
   `Dev Containers: Rebuild and Reopen in Container`。
3. 等待镜像构建和安装步骤结束。安装步骤以一次测试运行收尾，绿灯即代表环境可用。

## 装了什么

| 组件 | 说明 |
| --- | --- |
| Python | 3.11，与 CI 测试所用版本一致 |
| 软件包 | `litegrip-mujoco`，可编辑安装并带上 `.[dev]` extras |
| MuJoCo | 最新 3.x，附带 Mesa 软件 GL、GLX、EGL、OSMesa 与 X11 客户端库 |
| gh CLI、Node 22 | `gh` 用于 GitHub 操作，Node 用于 CI 的 markdown lint |

仿真不需要 GPU。夹爪模型很小，MuJoCo 在 CPU 上就跑得比实时快，查看器用 Mesa 软件渲染。

pip 已指向清华 TUNA 源（Dockerfile 里的 `PIP_INDEX_URL`）。网络能直连 PyPI 的话删掉那一行
即可。

## 常用命令

工作目录就是仓库根目录。

```bash
python -m pytest tests/ -v                                  # 测试套件
npx --yes markdownlint-cli2@0.23.3 "README.md" "README.zh-CN.md" "docs/*.md"   # CI 的 lint
python examples/01_hello_sim.py --no-render                # 例程，无图形环境
python examples/02_move_sim.py
python examples/03_trajectory.py
```

## 交互式查看器

MuJoCo 查看器窗口需要 Windows 宿主机上有一个 X server。容器已经替你设好
`DISPLAY=host.docker.internal:0.0`，所以装上 VcXsrv 窗口就能弹出来：

1. 从 <https://sourceforge.net/projects/vcxsrv/> 安装 VcXsrv。
2. 用 XLaunch 启动：Multiple windows，显示编号 0，勾选 **Disable access control**。
3. 启动 VcXsrv 后运行 `python examples/01_hello_sim.py` —— 查看器打开并开始动画。

没有 X server 在跑的时候，不要带查看器运行例程 —— `render=True` 会报 GLFW 错误。此时用
`--no-render`，或者删掉 `devcontainer.json` 里的 `remoteEnv` 块，只做无图形工作。

打开过查看器的运行可能在例程打印完成行之后以 `Segmentation fault` 收尾。那是主 README 里
描述的上游 GL 退出崩溃，不是仿真失败；`--no-render` 不受影响。

在 Linux 宿主机上，删掉 `remoteEnv` 块，把 `DISPLAY` 改成你自己的 X server 或 Wayland
会话。
