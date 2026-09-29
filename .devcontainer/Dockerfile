# Dev container image for litegrip-mujoco.
#
# Python 3.11 matches the CI test job. All Python dependencies are installed
# by postCreateCommand from pyproject.toml, so this file only carries the
# system packages the MuJoCo viewer needs: Mesa software GL (llvmpipe),
# GLX/EGL/OSMesa, and the X11 client libraries glfw links against.
FROM mcr.microsoft.com/devcontainers/python:3.11-bookworm

# Tsinghua TUNA mirror: direct PyPI access is slow from China. Override for
# networks closer to PyPI.
ENV PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple

# No GPU is required. The gripper model is tiny and MuJoCo runs it faster
# than real time on CPU; the viewer renders with llvmpipe over X11.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
        libgl1 \
        libgl1-mesa-dri \
        libglx-mesa0 \
        libegl1 \
        libegl-mesa0 \
        libosmesa6 \
        libglu1-mesa \
        libx11-6 \
        libxrandr2 \
        libxinerama1 \
        libxcursor1 \
        libxi6 \
        libxext6 \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*
