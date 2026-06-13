source /opt/ros/humble/setup.bash
source /opt/ros/humble/franka/setup.bash
ros2 launch franka_bringup franka.launch.py \
    arm_id:=fr3 \
    robot_ip:=172.16.0.3 \
    load_gripper:=true \
    use_fake_hardware:=false
