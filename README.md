# QD Radar2026
## 一键启动脚本
```bash
./start_radar.sh
```
## 手动启动脚本
### 1、相机外参标定
```bash
ros2 launch  tdt_vision calibrate_radar.launch.py
```
按Enter键开始标定,依次点击我方堡垒靠近我方的底角，我方前哨站血条底部，敌方基地引导灯，敌方前哨战引导灯，敌方四十三度坡围挡左底角。

每次点击后可使用wasd调节上下左右，按n键保存当前点，保存5个点后按Enter自动计算外参并保存在config/out_matrix.yaml

### 2、启动
```bash
#一定按步骤来
ros2 launch rm_serial_driver serial_driver.launch.py #启动裁判系统通信
ros2 launch livox_ros2_driver livox_lidar_launch.py #启动激光雷达驱动
ros2 launch tdt_vision radar.launch.py #启动相机
ros2 launch dynamic_cloud lidar.launch.py #启动激光雷达识别
ros2 run debug_map debug_map #启动地图可视化
ros2 topic pub -r 0.2 /match_info vision_interface/msg/MatchInfo "{self_color: 0, match_time: 180, robot_hp: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], marks: [0,0,0,0,0,0], ultimate: 0, eventtype: 0}" #test
```

## 可视化
Launch文件已集成foxglove-bridge,启动后直接打开foxglove-studio即可查看

