# 2.5D 静态航点规划器

入口是 `main.py`。规划器读取一次飞行 rosbag 的：

- DLIO-SLAM 最后一帧累计地图：识别 OCCUPIED 障碍物和墙壁；
- 逐帧 deskewed 点云：与 odom 位姿配合做激光射线清空；
- odom：提供每帧点云的射线原点。

规划平面为 `z` 上下 `slice_half_height` 米的 XOY 切片。没有射线观测的
栅格保持 UNKNOWN，A* 只允许进入 FREE 栅格；累计地图命中的栅格优先级高于
射线清空结果。

逐帧点云第一次解码后会写入 `planner.cache_dir`，缓存按 bag 文件指纹、输入话题、
frame 和 `scan_voxel_size` 校验；改变这些输入会自动重新读取，改变起终点或 A*
参数不需要重新扫描 bag。占据栅格也会按 bag 指纹和切片、飞机尺寸、栅格及地图
采样参数单独持久化，所以重新打开软件并选择同一 bag 时，通常连栅格构建也会跳过。

FREE 栅格会按 A* 使用的 8 邻域划分连通域。起点或终点落在 OCCUPIED 时，自动
吸附到能与另一端连通的 FREE 连通域；两端都需要吸附时，优先选择吸附总距离较小
的共同连通域，同距离优先面积较大的区域。UNKNOWN 不会被吸附，也不会作为通道。

当前默认 `map_voxel_size=0.4m`，用于保留场景覆盖并避免墙点在细栅格中被过度膨胀；
它仍然是面板中的可调参数。

模块职责：

- `bag_reader.py`：rosbag 读取、PointCloud2 解码、odom 插值；
- `sampling.py`：3D 体素稀疏采样；
- `occupancy.py`：三态栅格和射线清空；
- `planner.py`：A*、路径压缩和航点生成；
- `config.py`：YAML 配置加载、保存和另存；
- `tui.py`：curses 键盘参数面板；
- `xml_writer.py`：waypoint_manager XML 输出。

运行：

```bash
cd ~/uav_ground_station
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./scripts/plan_2d/main.py
```

面板中用方向键选择参数，`Enter/e` 编辑，`p` 规划，`x` 保存 XML，`w/a`
保存或另存配置，`l` 加载配置，`b` 选择 rosbag，`q` 退出。
