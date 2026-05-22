# Teleoperation Package

This package contains the teleoperation stack used for the dual-arm master/slave setup based on the MUR platforms and UR arms.

At the moment, the main real-world setup is centered on:

- `mur620b` as the master robot
- `mur620d` as the slave robot
- one left-arm channel and one right-arm channel

The reference launch file is:

```bash
roslaunch teleoperation master_slave_mur620b_mur620d_dual_real.launch
```

## Overview

The package is designed for bilateral teleoperation. The operator interacts with the master arms, while the slave arms track the commanded motion and return force information through the teleoperation controller.

The current package includes:

- master-side haptic control
- slave-side twist outer-loop control
- optional bimanual/object coupling utilities
- helper scripts for force/torque sensor zeroing
- rosbag recording and replay tools

## Main Launch File

Use the following launch file for the standard dual-arm real setup:

```bash
roslaunch teleoperation master_slave_mur620b_mur620d_dual_real.launch
```

This launch file starts the teleoperation pipeline for both left and right arms.

## Configuration Notes

The YAML files contain the common controller parameters for the two master arms and the two slave arms. Side-specific settings are then overridden in the launch file.

The configuration filenames can be misleading, so do not assume that the file names alone fully describe the effective runtime configuration.

The current code also contains setup-specific frame rotation compensation because the UR mounting on `mur620b` and `mur620d` is not identical. This compensation is defined in the main launch file and is not fully automatic.

If you change the robots, the mechanical mounting, or the reference frames, you must review and update these rotation parameters.

## Helper Launch Files

The following launch files are useful for moving the robots to predefined poses:

- `mur620b_mur620d_moveit_home_dual_arm.launch`
- `mur620b_mur620d_moveit_teleop_home_dual_arm.launch`

The second launch file moves the robots to a teleoperation-specific pose that was added in this branch through the SRDF configuration.

That pose is defined identically for both master and slave. Because the physical mounting differs between the two robots, they will not appear perfectly aligned at startup, but the teleoperation initialization will drive them toward a shared operating pose.

## Operational Constraints

The robots are sensitive only to the force/torque sensor measurements. In practice, this means the intended interaction is through the load cells at the tool side.

Do not expect correct behavior if force is applied directly along the robot kinematic chain, for example by pushing links or joints instead of interacting through the sensed tool interface.

## Recording and Replay Workflow

To record and replay a teleoperation run:

1. Start the teleoperation node with the main launch file.
2. Start rosbag recording:

```bash
./teleoperation/scripts/start_rosbag_dual_real_run_snapshot.sh <run_name>
```

3. Perform the motion you want to record, then stop the script with `Ctrl-C`.
4. Analyze the recorded bag and choose the replay window:

```bash
rosrun teleoperation analyze_dual_slave_replay_window.py /path/to/original.bag
```

5. Replay the recording:

```bash
roslaunch teleoperation replay_dual_slave_twist_from_bag.launch \
  manifest:=/path/to/replay_manifest.yaml
```

For more details, see:

- [Info_replay.md](./Info_replay.md)
- [Replay_spec.md](./Replay_spec.md)

## Important Tuning Note

The teleoperation node sends commands through the twist controller. If the twist-controller limits are too restrictive, the commanded motion will be clipped and the robot may not move as expected.

If the robot response feels slower, smaller, or more damped than intended, check the twist-controller velocity, acceleration, and filtering limits before changing the teleoperation logic.
