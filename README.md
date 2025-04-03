
# 自主无人机降落系统 Autonomous UAV landing

基于 ArUco 标记的无人机自主降落系统，包含相机标定、多尺度标记板生成、标记检测和降落控制等功能。

## 项目结构

```
.
├── create_board/          # ArUco 标记板生成工具
│   ├── create_aruco_board.cpp    # 生成 ArUco 标记板
│   └── create_aruco_marker.cpp   # 生成单个 ArUco 标记
├── detect_board/         # ArUco 标记检测模块
│   ├── src/             # 源代码
│   ├── include/         # 头文件
│   ├── param/          # 参数配置文件
│   └── lib/            # 依赖库
├── write_camera/        # 相机参数保存工具
├── Calibration_ZhangZhengyou_Method/  # 相机标定模块
│   ├── calib_IR.py     # 红外相机标定
│   └── calib_RGB.py    # RGB相机标定
└── Landing/            # 多标记检测降落控制模块
```

## 功能模块说明

### 1. 相机标定 (Calibration_ZhangZhengyou_Method)
- 使用张正友标定法进行相机标定
- 支持 RGB 和红外相机的标定
- 计算相机内参和畸变参数
- 标定结果保存为 XML 格式

### 2. 标记板生成 (create_board)
- 生成自定义 ArUco 标记板
- 支持生成单个 ArUco 标记
- 可配置标记大小、ID 和数量

### 3. 标记检测 (detect_board)
- 基于 OpenCV 的 ArUco 标记检测
- 实时位姿估计
- 支持多标记同时检测

### 4. 相机参数管理 (write_camera)
- 将相机标定参数保存为 XML 文件
- 方便参数管理和加载

### 5. 降落控制 (Landing)
- 基于 Prometheus 项目的多标记检测
- 实现精确的降落控制
- 支持多标记位姿融合

## 依赖项
- OpenCV
- Python 3.x
- CMake
- ROS (可选，用于降落控制)

## 使用说明
1. 首先进行相机标定，获取相机参数
2. 使用 create_board 生成所需的 ArUco 标记板
3. 运行标记检测程序进行实时检测
4. 配置降落控制参数，实现自主降落



