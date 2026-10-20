"""car_bridge —— X5 侧小车底盘桥接包。

模块：
    protocol      X5 <-> C8T6 串口帧（与固件同源，带黄金向量测试）
    bridge_node   /cmd_vel -> 串口帧 -> C8T6 -> CAN；回读 /car/status
    link_test     不依赖 ROS 的链路自检（看门狗 + 丢帧）
    teleop_keys   键盘遥控（纯 termios，不依赖 apt 包）
    record_path   建图时录轨迹
    follow_path   纯追踪回放走一圈
    map_from_cloud  cloud_slam 累积成 PCD + 俯视图
"""

__version__ = "0.1.0"
