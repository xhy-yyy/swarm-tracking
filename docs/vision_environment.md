# 视觉环境安装与自检

本指南说明六机 YOLO 检测的运行环境。所有仓库命令均从仓库根目录执行，
系统基线为 Ubuntu 20.04 + ROS Noetic。先按 [安装说明](installation.md)
安装系统依赖并构建工作空间，再执行本页中的 ROS 消息与权重检查。

## 两套 Python 环境的分工

| 环境 | 解释器 | 运行内容 |
| --- | --- | --- |
| 系统 ROS 环境 | `/usr/bin/python3`，Ubuntu 20.04 上为 Python 3.8 | catkin 构建、融合节点、ROS 工具、行人/裁判、`show_swarm_cams.py` 显示节点。 |
| 视觉环境 | Conda `swarm-vision`，Python 3.10 | `swarm_vision.py` 中的 YOLO 批量检测、图像解析、目标定位与标注。 |

[`run_swarm_vision.sh`](../src/fly_px4/scripts/run_swarm_vision.sh) 使用
`VISION_PYTHON` 显式运行检测脚本；其他 catkin Python 节点使用构建时的
系统解释器。ROS 构建与任务启动终端应保持在系统 ROS 环境中，无需激活视觉环境。

[`scripts/env.sh`](../scripts/env.sh) 加载 ROS 与本工作空间的环境，
使视觉解释器能通过 `PYTHONPATH` 导入 `rospy` 和 catkin 生成的 `track_msgs`。
这不会把 Python 3.10 变成系统 Python，也不需要向 Conda 中重新安装整套 ROS。

检测节点直接用 NumPy 解析 `sensor_msgs/Image`，不调用 `cv_bridge`。
六路显示节点 `show_swarm_cams.py` 则在系统 Python 3.8 中使用系统 OpenCV
与 Noetic 的 `cv_bridge`，避免跨解释器混用编译好的扩展。

## 依赖与版本

以下来自 [`requirements-vision.txt`](../requirements-vision.txt)，
是原视觉环境的依赖记录。它不包含原显卡型号、驱动版本或 PyTorch 的 CUDA
构建标签，因此不能据此推导某个固定显卡、显存或驱动配置。

| 依赖 | 记录版本 | 作用 |
| --- | --- | --- |
| Python | `3.10` | 与系统 ROS Python 分开的视觉解释器。 |
| `torch` | `2.12.0` | YOLO 模型推理、GPU/CPU 张量运算。 |
| `torchvision` | `0.27.0` | 与 PyTorch 配套的视觉算子。 |
| `ultralytics` | `8.4.57` | `YOLO` 模型加载与预测接口。 |
| `numpy` | `2.2.6` | RGB/深度数组、几何计算。 |
| `opencv-python` | `4.13.0.92` | 视觉环境中的 `cv2`，用于检测标注。 |
| `scipy` | `1.15.3` | 数值计算依赖。 |
| `PyYAML` | `6.0.3` | YAML 配置与 ROS Python 依赖。 |
| `rospkg` | `1.6.1` | ROS 包路径与 Python 支持。 |

## 创建视觉环境

如果尚未安装 Conda，先按 [Conda 官方 Linux 安装说明](https://docs.conda.io/projects/conda/en/stable/user-guide/install/linux.html)
完成安装与终端初始化；保留系统 `/usr/bin/python3` 供 ROS 使用。

### 方式 A：使用仓库环境文件

已安装 Conda 时，执行：

```bash
conda env create -f environment.yml
conda activate swarm-vision
python -m pip check
python -c "import sys; print(sys.executable)"
```

[`environment.yml`](../environment.yml) 创建 Python 3.10 环境，并通过 pip
安装依赖文件中的固定版本。记下最后输出的解释器绝对路径，后面用于配置
`VISION_PYTHON`。如果默认下载的 PyTorch 构建不适合本机 GPU，使用方式 B。

### 方式 B：先选择 PyTorch 的 GPU 或 CPU 构建

本方式与方式 A 二选一。先创建空的视觉环境：

```bash
conda create -n swarm-vision python=3.10 pip -y
conda activate swarm-vision
python -m pip install --upgrade pip
```

使用 NVIDIA GPU 时，先运行 `nvidia-smi` 检查显卡与驱动，再依据
[PyTorch 固定版本安装表](https://pytorch.org/get-started/previous-versions/#v2120)
和 [NVIDIA 驱动兼容说明](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html)
选择安装源。下面是官方列出的 **CUDA 12.6 构建示例**，仅在显卡架构与驱动
支持该构建时使用：

```bash
python -m pip install torch==2.12.0 torchvision==0.27.0 --index-url https://download.pytorch.org/whl/cu126
```

`nvidia-smi` 中的 CUDA 版本表示驱动可支持的 CUDA 版本，不能作为当前
PyTorch wheel 使用的运行时版本；后者通过 `torch.version.cuda` 查看。
字段含义见 [NVIDIA `nvidia-smi` 文档](https://docs.nvidia.com/deploy/nvidia-smi/index.html#cuda-version)。

仅使用 CPU 调试时，改用下面这一条，不同时安装两种构建：

```bash
python -m pip install torch==2.12.0 torchvision==0.27.0 --index-url https://download.pytorch.org/whl/cpu
```

选定构建后安装其余依赖并检查一致性：

```bash
python -m pip install -r requirements-vision.txt
python -m pip check
python -c "import sys; print(sys.executable)"
```

先安装与硬件匹配的 PyTorch、再安装 Ultralytics 的做法也见
[Ultralytics 官方安装说明](https://docs.ultralytics.com/quickstart/)。
`torch` 和 `torchvision` 应保持配套版本与相同的 GPU/CPU 构建来源。

## 配置解释器与权重

首次使用时创建本地配置：

```bash
cp -n config/runtime.env.example config/runtime.env
```

编辑 `config/runtime.env`，把 `VISION_PYTHON` 替换为前面实际输出的路径：

```bash
VISION_PYTHON="/absolute/path/to/swarm-vision/bin/python"
# 可选：使用其他位置的兼容权重时设置。
# SWARM_WEIGHTS="/absolute/path/to/best_11s_new.pt"
```

`config/runtime.env` 已被 Git 忽略。未设置 `SWARM_WEIGHTS` 时，默认读取
`assets/weights/best_11s_new.pt`。该文件通过独立权重资源包提供，不在源码 Git 中。

权重类别顺序须为 `green, blue, brown, white, red`；两个红色行人由融合节点
区分航迹，不是两个检测类别。训练来源与许可信息见 [模型说明](model_card.md)。

## 启动前自检

以下检查使用**未激活 Conda 的 ROS 终端**。如果当前终端激活了视觉环境，
先退出该环境，再确认 ROS 使用的是系统 Python；不要在视觉环境中重新构建 catkin。

### 1. 检查 ROS 消息、视觉依赖与 CUDA

先完成 `bash scripts/build.sh`，然后在仓库根目录执行：

```bash
source scripts/env.sh
"$VISION_PYTHON" - <<'PY'
import sys
import cv2
import numpy as np
import scipy
import torch
import torchvision
import ultralytics
import yaml
import rospkg
import rospy
from sensor_msgs.msg import Image
from track_msgs.msg import DetectionArray

print("Python:", sys.executable, sys.version.split()[0])
print("torch / torchvision:", torch.__version__, torchvision.__version__)
print("ultralytics / OpenCV:", ultralytics.__version__, cv2.__version__)
print("NumPy / SciPy:", np.__version__, scipy.__version__)
print("CUDA available:", torch.cuda.is_available())
print("PyTorch CUDA runtime:", torch.version.cuda)
device = "cuda:0" if torch.cuda.is_available() else "cpu"
if device.startswith("cuda"):
    print("GPU:", torch.cuda.get_device_name(0))
    print("Compiled GPU architectures:", torch.cuda.get_arch_list())
x = torch.ones(1, device=device)
print("Tensor operation:", (x + x).item(), "on", device)
print("ROS messages and vision imports OK")
PY
```

导入检查和张量运算不需要启动 ROS master 或 Gazebo。`CUDA available: True`
表示 PyTorch 能发现 CUDA；张量运算进一步检查实际设备执行。
`torch.version.cuda` 表示该 PyTorch 构建所使用的 CUDA 运行时，CPU 构建会返回 `None`。

### 2. 检查权重并执行一次推理

安装与当前源码匹配的权重资源后，在同一终端执行：

```bash
python3 scripts/check_assets.py --weights-only
"$VISION_PYTHON" - <<'PY'
import os
from pathlib import Path
import numpy as np
import torch
from ultralytics import YOLO

weights = Path(os.environ["SWARM_WEIGHTS"])
assert weights.is_file(), f"Missing checkpoint: {weights}"
model = YOLO(str(weights))
names = model.names
names = [names[i] for i in range(len(names))] if isinstance(names, dict) else list(names)
assert names == ["green", "blue", "brown", "white", "red"], names
device = "cuda:0" if torch.cuda.is_available() else "cpu"
image = np.zeros((480, 640, 3), dtype=np.uint8)
results = model.predict(image, device=device, half=device.startswith("cuda"), verbose=False)
print("Checkpoint:", weights)
print("Classes:", names)
print("Single-frame inference OK:", len(results), "result on", device)
PY
```

空白图用于检查权重加载、类别顺序与推理算子是否可运行，不评估检测准确率，
也不验证六路相机的实际吞吐。自定义权重需满足同一类别约定；资源校验脚本
检查的是仓库清单中的发布权重，自定义权重的哈希不会自动加入该清单。

### 3. 启动任务

完成仿真、行人与裁判启动后，在系统 ROS 终端运行：

```bash
source scripts/env.sh
bash src/fly_px4/scripts/start_swarm.sh base1
```

视觉节点默认使用 `cuda:0`、半精度推理、最多六路批量输入。CUDA 不可用时，
当前代码会打印警告、回退到 CPU 并关闭半精度；此时不保证达到配置的推理频率。
完整三终端启动顺序见 [安装与运行](installation.md#启动)。

## 常见问题

| 现象 | 检查方法 |
| --- | --- |
| `Vision Python not found` | 检查 `config/runtime.env` 中是否填写了真实解释器路径；用 `"$VISION_PYTHON" --version` 确认。 |
| `No module named rospy` | 先在系统 ROS 终端 `source scripts/env.sh`，再调用视觉解释器；检查 `/opt/ros/noetic` 是否已安装。 |
| `No module named track_msgs` | 先构建本工作空间，再重新加载 `scripts/env.sh`；确认没有混入其他 ROS 工作空间。 |
| `CUDA available: False` | 检查 `nvidia-smi`、PyTorch 是否为 CPU 构建、驱动兼容性和 GPU 可见性；在容器中运行时还要检查 GPU 映射。 |
| CUDA 架构不支持或 `no kernel image` | 对照显卡架构、PyTorch 构建支持的架构列表与官方安装表选择兼容构建。 |
| `torchvision::nms` 不存在或无法加载算子 | 检查 `torch` / `torchvision` 配套版本及 GPU/CPU 构建是否一致，按同一官方索引重新安装配套版本。 |
| `cv_bridge`、`multiarray` 或 `_ARRAY_API` 导入错误 | 检查报错扩展来自哪套 Python；显示节点保留在系统 Python 3.8 中，不在视觉 Python 3.10 中导入系统 `cv_bridge`。 |
| 权重不存在或类别不匹配 | 检查 `SWARM_WEIGHTS` 指向的文件与 `model.names`，并按模型说明核对五类顺序。 |
| 只有检测数据，没有标注窗口 | 检查图形桌面与 `DISPLAY`；显示节点依赖系统 OpenCV 和 `cv_bridge`，`annotated:=false` 会关闭标注和查看窗口。 |

无显示器调试时，可以按 [系统结构](architecture.md) 分别启动任务组成部分，
并为 `swarm_track.launch` 传入 `annotated:=false`；不要同时运行完整的
`start_swarm.sh` 与重复的同名视觉节点。
