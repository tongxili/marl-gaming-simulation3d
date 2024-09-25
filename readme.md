# MAPPO算法在无人机集群围捕场景下的三维仿真
演示三维场景下的多无人机集群围捕任务，蓝方保持固定队型前往目标点，红方使用MAPPO算法生成策略对蓝方进行围捕。

## 环境安装
使用了基于ROS+Gazebo的XTDrone仿真环境，使用前需要按照教程配置XTDrone仿真平台：https://gitee.com/robin_shaun/XTDrone

## 文件结构与功能
```
+---for_test
|   |   print_pose.py
|   |   red_observation.py
|   |  
|   \---leader_follower
|           ***
|           run_rule_agents.sh
|
+---MAPPO_3D
|       ***
|       MAPPO_Main.py
|
\---world
        multi_vehicle_6v6.launch
        outdoor2_new.world
```

* for_test/ 文件夹是进行初步测试的文件（可不用）    
    *  for_test/leader_follower/ 文件夹能演示蓝方固定编队前往目标点，红方随机速度飞行的三维场景。
    整个文件夹放在`XTDrone/coordination/` 下
    使用时执行`run_rule_agents.sh` 脚本即可展示效果

* MAPPO_3D/ 文件夹下是进行三维演示所需的文件
整个文件夹放在`XTDrone/coordination/` 下
使用时执行`MAPPO_Main.py` 脚本即可展示效果

* world/ 文件夹下是Gazebo中生成三维场景所需的文件，其中：
    * `multi_vehicle_6v6.launch` 是打开仿真环境的文件
    放在`px4_firmware/launch` 文件夹下
    * `outdoor2_new.world` 是添加了目标点的world场景
    放在`PX4_Firmware/Tools/sitl_gazebo/worlds` 文件夹下

## 使用流程
1. 打开launch文件
    ```bash
    cd ~/PX4_Firmware
    roslaunch px4 multi_vehicle_6v6.launch
    ```
2. 建立通信
    ```bash
    cd ~/XTDrone/communication
    bash multi_vehicle_communication.sh
    ```
3. 获取ground truth位姿
    ```bash
    cd ~/XTDrone/sensing/pose_ground_truth
    python get_local_pose.py iris 12
    ```
4. 获取target位置
    ```bash
    cd ~/XTDrone/coordination/red_vs_blue
    python get_target_pose.py
    ```
5. 键盘控制无人机起飞，到一定高度后悬停
    ```bash
    cd ~/XTDrone/control/keyboard
    python multirotor_keyboard_control.py iris 12 vel
    ```
6. 启动MAPPO仿真
    ```bash
    cd ~/XTDrone/coordination/red_vs_blue/MAPPO_3D
    python MAPPO_Main.py
    ```
    或启动leader-follower脚本
    ```bash
    cd ~/XTDrone/coordination/red_vs_blue/leader-follower
    bash run_rule_agents.sh iris 6 6 {formation_mode}
    ```