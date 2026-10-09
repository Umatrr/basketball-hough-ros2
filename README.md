# 基于霍夫圆检测的 ROS 2 篮球识别（纯霍夫测试分支）

独立的 ROS 2 Jazzy 工作空间。摄像头图像只经过霍夫圆检测和圆周边缘验证，不使用 HSV 颜色筛选。输出 `/basketball/detection`，并在窗口显示结果。`/basketball/mask` 是用于观察 Canny 圆周边缘的诊断图，不用于颜色筛选。

## 环境与运行

需要 ROS 2 Jazzy、`colcon`、`cv_bridge`、`python3-opencv` 和可用摄像头。默认读取 `/dev/video0`，图像大小 640×480。在工作空间根目录运行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch basketball_cv basketball_system.launch.py
```

自定义参数文件：

```bash
ros2 launch basketball_cv basketball_system.launch.py params_file:=/absolute/path/to/params.yaml
```

默认参数见 [`params.yaml`](src/basketball_cv/config/params.yaml)。修改后重启 launch。按 Ctrl+C 结束。

想按代码执行顺序理解三个节点、霍夫变换、候选评分和消息流，可阅读[完整代码逻辑](docs/code-walkthrough.md)。

## 检测步骤

1. 读取 BGR 图像，转灰度并高斯模糊，减轻球面纹理和噪声干扰。
2. `cv2.HoughCircles` 对边缘投票，给出圆心与半径。它不依赖橙色 mask 连通，因此深色缝线无需形态学填补。
3. 沿候选圆周采样边缘，计算圆周支持率。
4. 候选通过可见比例和边缘比例后，直接以 `edge_fraction` 作为分数，选择最高分。
5. 发布圆心、半径、裁剪到画面内的外接框、得分和带原图时间戳的 mask。

`confidence` 是规则评分，不是经过标定的识别概率。球被画面截断太多、失焦、严重反光，或者圆周与背景对比很低时，霍夫圆可能无法检出。

## 调参顺序

| 参数 | 调整建议 |
| --- | --- |
| `min_radius` / `max_radius` | 先按球的实际像素半径缩小搜索范围；近距离球可能很大，`max_radius: 0` 表示不限。 |
| `param2` | 漏检时逐步降低，如 30 → 26；误检多时提高。 |
| `param1` | 边缘太弱时降低；杂乱边缘多时提高。 |
| `blur_kernel` | 缝线或纹理影响投票时加大；轮廓被模糊时减小。 |
| `min_edge_fraction` | 人脸等圆形误检时提高；球的外缘很弱时降低。 |
| `min_visible_fraction` | 画面边缘的近距离篮球漏检时降低。 |
| `min_score` | 最后微调整体接受阈值；优先先调前面的验证条件。 |

`dp` 控制霍夫累加器分辨率，`min_dist` 控制候选圆心之间的最小距离。本分支只使用圆周边缘，查看 `/basketball/mask` 可判断 Canny 边缘是否连续。

检测算法在 [`circle_detection.py`](src/basketball_cv/basketball_cv/circle_detection.py)；ROS 节点在 [`detector_node.py`](src/basketball_cv/basketball_cv/detector_node.py)。摄像头和显示节点保留独立进程，由 [`basketball_system.launch.py`](src/basketball_cv/launch/basketball_system.launch.py) 一次启动。

## 分支说明

当前是 `hough_only_test` 分支：评分只有圆周边缘支持率，颜色不参与候选过滤或权重。适合确认霍夫几何本身能否找到篮球，但手部、背景圆形物体的误检可能增加。

- `main` / `hough_plan_adapted_to_bright_env`：亮环境版，保留较宽 HSV。
- `hough_plan_adapted_to_dark_env`：暗环境版，使用更严格的 HSV。
- `hough_only_test`：当前纯霍夫测试版。

切换后重新构建或使用 `--symlink-install` 的工作空间源码，并重新启动 launch。
