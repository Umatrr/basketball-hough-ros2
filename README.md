# 基于霍夫圆检测的 ROS 2 篮球识别

独立的 ROS 2 Jazzy 工作空间。摄像头图像经过霍夫圆检测，再由颜色和圆周边缘验证候选圆。输出 `/basketball/detection`，并在窗口显示结果。`/basketball/mask` 是用于观察橙色覆盖区域的诊断图，不用于寻找圆。

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
3. 沿候选圆周采样边缘，计算圆周支持率；在圆内部 85% 范围计算橙色像素比例。
4. 两项比例必须分别达到最低值；通过后按 `0.65 × 圆周支持率 + 0.35 × 橙色比例` 评分，选择最高分。
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
| `min_color_fraction` | 暗缝、遮挡造成漏检时降低；肤色误检时提高，或收紧 HSV。 |
| `min_visible_fraction` | 画面边缘的近距离篮球漏检时降低。 |
| `min_score` | 最后微调整体接受阈值；优先先调前面的验证条件。 |

`dp` 控制霍夫累加器分辨率，`min_dist` 控制候选圆心之间的最小距离。HSV 的 `h_min/h_max` 为色相范围，`s_min` 排除灰白区域，`v_min/v_max` 排除过暗和过亮区域。查看 `/basketball/mask` 可判断颜色参数是否适合当前光线。

检测算法在 [`circle_detection.py`](src/basketball_cv/basketball_cv/circle_detection.py)；ROS 节点在 [`detector_node.py`](src/basketball_cv/basketball_cv/detector_node.py)。摄像头和显示节点保留独立进程，由 [`basketball_system.launch.py`](src/basketball_cv/launch/basketball_system.launch.py) 一次启动。
