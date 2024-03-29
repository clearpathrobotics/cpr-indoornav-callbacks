CPR Indoornav Callbacks
========================

This package contains Python modules used to implement custom callbacks at specific endpoints on the map
when the point in question is reached.

Dependencies
-------------

This package depends on:
- `video_recorder_msgs`: https://github.com/clearpathrobotics/video_recorder
- `ptz_control_msgs`: _currently unreleased publicly_
- `cpr_gps_navigation_msgs`: _currently unreleased publicly_


Setup
------

In addition to the normal setup steps for IndoorNav, make sure to add

```bash
# enable python callbacks on endpoints
export STRATEGY_MANAGEMENT_ENABLE_REMOTE_CODE_EXECUTION_STEP=1
```

to `/etc/ros/setup.bash` on the Nav PC. Otherwise the callbacks will not execute.
