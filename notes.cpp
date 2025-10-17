/*

SSID: Vicon_HAMR
Pwd: hamr@hamr

Pi: hamr@10.42.0.14
password: hamr 

HAMR EKF Step:

HAMR_Base/odom: vicon publisher
/odom: we made to test odom

HAMR_Base/pose: vicon publisher subset of odom, 
/robot_pose: we made to check with pose

test vicon:
ros2 launch mocap_vicon vicon_launch.py    // launch vicon
ros2 run hamr_control hamr_vicon_trace  //plotting 



check on hamr_base, hamr_turret on vicon PC, seconds need to run or press seconds to run again




1) init_odometry(), create covariance matrix P:

    covariance = diag([0.01, 0.01, 0.01])

set robot x,y,yaw = 0:

    float robot_x = 0.0;                        // robot position in meters
    float robot_y = 0.0;
    float robot_theta = 0.0;                    // robot orientation radians

2) init imu configurations
    cfg.R_yaw_rad2 = sq(12.0f * M_PI / 180.0f); // imu yaw measurement variance
    cfg.gate_sigma = 3.0f;  // gating gamma value
    cfg.alignment_timeout_ms = 5000.0f; //time between auto realignments
    cfg.min_calibration_level = 2;      //Required IMU self-calibration level before using yaw
    cfg.enable_periodic_realignment = true; //call realignment
    cfg.realignment_threshold = 30.0f * M_PI / 180.0f; // 30 degrees
    cfg.realignment_count_threshold = 10;

3) setup called imu_task, which calls imu_update() every 100 ms
- sens.getRPY(r,p,y) 
- stores yaw in global shared variables g_yaw_latest, g_yaw_latest_us, g_yaw_valid

4) Locallization block begins, updates every 100ms to call updateOdometry()

  updateOdometry():
    1. compute dt: t_now - t_last
    2. get current ticksL and ticksR, sets current to previous in the end
    3. 


*/