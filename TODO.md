# TODO

## MUR

- controller_manager general (mur_base.launch) or for each ur (ur.launch)?

## Lift

lift in mur launch, but not mir

- load lift_module in mur.lauch
- connect left_lift_bottom (mir) ->lift_top -> UR or left_lift_bottom (mir) ->ur in mur.launch

## HW/Sim identic

same urdf for hw and sim:

- remap robot_description
- robot_state_publisher
    - change tf_prefix to ""
- joint_state_publisher
    - change tf_prefix to "UR16_l/"
- call joints "mir/...", UR16_l/..." in urdf and tf_prefix="mur"?
    - urdf: mir links=/mir/..., therefore tf_prefix="mur"
- only one robot_state_publisher for all mur?
    - remap /mur/ur/joint_states to /mur/joint_states
        - or use **relay** to publish to both and remap old tf
    - disable mir: remove_tf_frames, /mur620b/mir/tf_remove_state_publisher_frames
- controller_config_file: load general config file and only rename joints?
- enable launch of move_group in dual_ur.launch

### MoveiIt one process

use only one process for both arms.

- mur urdf
- has to be executed twice:
    - grip_service_interface
    - bringup.launch
    - ur_calibrated_pose_pub.launch
    - ur_twist_limiter

- has to be executed once:
    - move_group (mur_moveit_config package)
