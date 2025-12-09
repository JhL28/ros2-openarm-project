[cite_start]这是一个非常专业和完整的 `README.md` 模板，完全符合 ROS2 课程设计论文的要求 [cite: 5, 63]。它结构清晰，涵盖了技术细节、安装说明和团队分工，可以直接使用。

我已对其进行了以下方面的优化和修改：

1.  **分支规范补充：** 增加了上一回合讨论的 **分支规范** 章节，这对于多人协作非常关键。
2.  **团队分工细化：** 将分工表与详细贡献结合，方便团队成员对照。
3.  **格式调整：** 确保 Markdown 格式统一且易读。

-----

## ✅ 最终版 `README.md`

```markdown
# ROS2 + OpenArm 双臂系统：视觉感知 · 手眼标定 · 运动规划 · 抓取系统

本项目基于 **ROS2 (Humble/Foxy)**、**OpenArm 双臂机器人**、**MuJoCo/Gazebo 仿真环境** 与 **深度相机**，实现从  
**物体检测 → 三维定位 → 手眼标定 → 双臂轨迹规划 → 抓取执行** 的完整智能机器人系统。

---

## 📘 项目简介

本系统实现以下完整闭环流程：

1.  深度相机采集 RGB/Depth 数据  
2.  图像/点云预处理  
3.  物体检测（传统方法 + 深度模型支持）  
4.  目标三维定位（坐标变换 camera → base）  
5.  手眼标定（Tsai-Lenz / $AX = XB$）  
6.  MoveIt2 运动规划（双臂协同工作）  
7.  夹爪控制与抓取状态机  
8.  多轮抓取实验统计与误差分析  
9.  MuJoCo 仿真与实物平台对比验证（若具备实物设备）

---

## 🌳 Git 分支工作流规范

本项目采用简化版 GitFlow 工作流，所有代码需遵循以下分支规范，以保证代码稳定性：

* **`main` (主分支):** 仅存放经过完整测试、稳定且可直接运行的发布版本代码。**禁止直接向此分支提交。**
* **`dev` (开发主线):** 所有 `feature` 分支合并的目标，集中了所有最新功能。所有功能开发都必须基于此分支创建。
* **`feature-xxx` (功能分支):** 针对特定任务创建的开发分支（例如 `feature-visual-detection`）。

---

## 🏗 系统结构图

（请将您的系统架构图放在 `docs/system_architecture.png` 并更新此占位符）

```

相机 → 感知模块 → 物体3D定位 → 手眼标定 → 运动规划器 → 控制器 → 夹爪执行

````

---

## 🔧 功能模块与分工

本项目由四位成员协作完成，分工如下：

| 模块 | 功能说明 | 核心贡献者 | 论文章节对应 |
| :--- | :--- | :--- | :--- |
| **模型与仿真** | URDF/Xacro 扩展、相机模型、MuJoCo 场景构建 | **成员2** | 4.2, 5.2, 5.6 |
| **视觉感知** | 图像/深度处理、物体检测、三维定位 | **成员3** | 5.4 |
| **手眼标定** | 标定数据采集、AX=XB 求解、误差评估 | **成员4** | 5.3 |
| **运动规划** | MoveIt2 轨迹规划、双臂协作策略 | **成员4** | 5.5 |
| **抓取执行** | 夹爪控制、抓取状态机、成功率统计 | **成员4** | 5.5, 6 |
| **系统集成** | 接口设计、TF 框架、总控 launch、文档撰写 | **成员1** | 5.1, 7, 8 |

---

### 👥 团队成员详细贡献

以下是各成员的具体职责和主要代码产出：

#### **成员1 — 系统架构与集成**
* [cite_start]**职责：** 系统整体架构设计 [cite: 23]、ROS2 接口定义、TF 框架管理。
* **代码：** 总控 `mujoco_full_system.launch.py`、集成文档 `interfaces.md`。

#### **成员2 — 机械模型与仿真**
* [cite_start]**职责：** OpenArm 机器人的硬件模型扩展 [cite: 24][cite_start]、MuJoCo 环境建模 [cite: 26]。
* **代码：** `urdf/openarm_with_camera.xacro`、`mujoco/mujoco_world.xml`。

#### **成员3 — 视觉感知与定位**
* [cite_start]**职责：** 相机数据处理、物体检测算法实现、物体坐标系转换 [cite: 33]。
* **代码：** `perception/src/object_detector.py`、`perception/src/pointcloud_processing.py`。

#### **成员4 — 手眼标定与规划抓取**
* [cite_start]**职责：** 手眼标定求解与验证 [cite: 29][cite_start]、MoveIt2 运动规划 [cite: 35][cite_start]、抓取控制逻辑 [cite: 36]。
* **代码：** `calibration/scripts/*`、`motion_control/src/motion_planner.py`、`motion_control/src/grasp_state_machine.py`。

---

## 📦 环境依赖

### 必要软件
-   **ROS2** Humble / Foxy
-   **Python** $\ge 3.8$
-   **MoveIt2** (运动规划)
-   **MuJoCo** 2.x / mujoco_ros / ros_gz (物理仿真)
-   **PCL** / **OpenCV** (点云和图像处理)

### 安装命令示例

```bash
# 安装必要的ROS2包
sudo apt install ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-moveit

# 安装Python依赖
pip install opencv-python numpy transforms3d
````

-----

## ⚙ 安装与构建

```bash
# 1. 创建工作空间 (如果尚未创建)
mkdir -p ~/ros2_openarm_ws/src
cd ~/ros2_openarm_ws/src

# 2. 克隆本项目代码
git clone <本仓库地址>

# 3. 回到工作空间根目录
cd ..

# 4. 安装所有依赖项
rosdep install --from-paths src -r -y --ignore-src

# 5. 编译工作空间
colcon build

# 6. 设置环境变量
source install/setup.bash
```

-----

## 🚀 系统运行说明

### ▶ 启动完整仿真系统

```bash
ros2 launch system_bringup mujoco_full_system.launch.py
```

启动内容包含：

  * OpenArm 双臂机器人
  * RGBD 相机
  * 桌子与物体模型
  * 感知节点
  * 手眼标定模块
  * 运动规划和控制器

-----

### ▶ 单独运行视觉感知节点

```bash
ros2 launch perception perception.launch.py
```

查看检测结果（应包含物体坐标）：

```bash
ros2 topic echo /detected_objects
```

-----

### ▶ 手眼标定流程

采集样本（机械臂移动到预设位姿并记录相机位姿）：

```bash
ros2 run calibration collect_samples
```

计算标定矩阵 ($AX=XB$)：

```bash
ros2 run calibration compute_hand_eye
```

结果输出到 `results/` 目录：

  * `results/calibration_result.yaml`
  * `results/calibration_error.csv`

-----

### ▶ 抓取规划与执行

```bash
ros2 launch motion_control grasp.launch.py
```

抓取状态机流程：

```
APPROACH → PREGRASP → CLOSE_GRIPPER → LIFT → RETRACT
```

-----

## 📁 项目目录结构

```text
.
├── launch/
│   ├── mujoco_full_system.launch.py  # 完整系统启动文件
│   ├── perception.launch.py
│   ├── calibration.launch.py
│   └── motion_control.launch.py
│
├── urdf/
│   ├── openarm_with_camera.xacro      # 集成相机后的机器人模型
│   ├── apple.xacro                    # 物体模型
│   ├── banana.xacro
│   └── table.xacro                    # 环境模型
│
├── mujoco/
│   └── mujoco_world.xml               # MuJoCo 场景配置
│
├── perception/                        # 视觉感知模块
│   ├── src/object_detector.py
│   ├── src/pointcloud_processing.py
│   └── config/perception_params.yaml
│
├── calibration/                       # 手眼标定模块
│   ├── scripts/collect_samples.py     # 采集样本脚本
│   └── scripts/compute_hand_eye.py    # 求解 AX=XB 脚本
│
├── motion_control/                    # 运动控制与抓取执行
│   ├── src/motion_planner.py
│   ├── src/gripper_controller.py
│   └── src/grasp_state_machine.py     # 抓取逻辑状态机
│
├── docs/                              # 文档、设计与报告
│   ├── system_architecture.pdf
│   ├── interfaces.md
│   └── hardware_setup.pdf
│
├── results/                           # 实验数据与结果
│   ├── perception_accuracy.csv
│   ├── grasp_success.csv
│   └── calibration_error.csv
│
└── README.md
```

-----

## 🎬 成果展示

（请将项目截图和视频链接放在这里，以提高可读性。）

  * 🍎 **物体检测效果图：**
  * 🤖 **抓取演示截图：**
  * 📈 **标定误差曲线：**
  * 📊 **抓取成功率统计：**

（建议将资源放在 `media/` 目录）

-----

## 📚 参考文献

1.  ROS2 Documentation
2.  MoveIt2 Documentation
3.  MuJoCo Documentation
4.  Tsai R. & Lenz R., “A New Technique for Hand-Eye Calibration”
5.  PCL / OpenCV 官方文档

-----

## 📝 License

MIT License. 可在科研、教学与学习场景中自由使用与修改。

```
```
