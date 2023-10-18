# TODO

## check if dual_ur newest commit is correct
- tf_prefix and ur_prefix. Working to set commadns?
- why is tf_prefix of twist_commands not mur620b/... but mur620/...

## disable state pub of ur!
- right now just disabled in sub package
- see match_mobile_robotics/ur/ur_ros_driver/ur_robot_driver/launch/ur_common.launch

## HW/Sim identic

### Twist-Controller:
tool0_controller tf is now remapped to /tf_old
- -> change name from UR10_r/base -> UR10_r/tool0_controller to /mur620/UR10_r/... and publish to /tf


### Set Caster_joints
- to 0 or fixed
- otherwise always warning from move_group
