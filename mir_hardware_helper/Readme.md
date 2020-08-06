# Mir hardware helper
This package implements helper for the Mir plattform. For using the package execute the mir.launch file while
passing its arguments.

# Attention
This package uses the mir bridge from the [mir package](https://github.com/dfki-ric/mir_robot/tree/melodic/mir_driver/src/mir_driver). Since there are tf_prefix problems within this package the header frame stampes of published sensor data must be used with caution! Reason for this is the missing tf_prefix arg within the mir_bridge in [mir_driver](./launch/mir.launch).