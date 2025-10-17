#include <gtest/gtest.h>
#include "Arduino.h"
#include "esp32-hal.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include "WiFi.h"
#include "WiFiUdp.h"

#include "ekf_localization.h"
#include "odometry.h"



// reset shared globals
static void reset_globals(){
  extern float robot_x, robot_y, robot_theta;
  extern float sampled_x, sampled_y, sampled_theta;
  extern bool first_update;
  extern unsigned long last_update_time;
  extern float covariance[6];
  robot_x = robot_y = robot_theta = 0.f;
  sampled_x = sampled_y = sampled_theta = 0.f;
  first_update = true;
  last_update_time = millis();
  for(int i=0;i<6;++i) covariance[i]=0.f;

  ticksL = ticksR = 0;

  extern bool g_aligned;
  extern float g_yaw_offset;
  extern unsigned long g_last_realignment_ms;
  extern int g_large_innovation_count;
  g_aligned = false;
  g_yaw_offset = 0.f;
  g_last_realignment_ms = 0;
  g_large_innovation_count = 0;
}

TEST(MathUtils, WrapToPi){
  EXPECT_NEAR(wrapToPi(0.0f), 0.0f, 1e-6);
  EXPECT_NEAR(wrapToPi(3.1415927f), 3.1415927f, 1e-6);
  EXPECT_NEAR(wrapToPi(-3.1415927f), -3.1415927f, 1e-6);
  EXPECT_NEAR(wrapToPi(3.0f*3.1415927f), -3.1415927f, 1e-6);
  EXPECT_NEAR(wrapToPi(-3.0f*3.1415927f), 3.1415927f, 1e-6);
}

TEST(EKFYaw, InitialAlignAndInnovation){
  reset_globals();

  extern float robot_theta;
  robot_theta = 30.0f * (float)M_PI / 180.0f;

  EkfYawConfig cfg;
  cfg.enable_periodic_realignment = true;
  // cfg.realignment_threshold = 5.0f * (float)M_PI/180.0f; // 5 deg
  // cfg.realignment_count_threshold = 3;
  // cfg.alignment_timeout_ms = 50;
  cfg.realignment_threshold = 2.0f * (float)M_PI/180.0f;
  cfg.realignment_count_threshold = 1;  // or 2 if you prefer
  cfg.alignment_timeout_ms = 0;         // or small like 5–10ms with your clock steps

  float imu_yaw = 10.0f * (float)M_PI / 180.0f;

  ASSERT_TRUE(ekfYawUpdate(imu_yaw, cfg));
  EXPECT_NEAR(g_yaw_offset * 180.0f / (float)M_PI, 20.0f, 0.5f);

  robot_theta += 10.0f * (float)M_PI/180.0f;
  ASSERT_TRUE(ekfYawUpdate(imu_yaw, cfg));
}

TEST(EKFYaw, RealignmentOnLargeInnovationBurst){
  reset_globals();

  extern float robot_theta;
  robot_theta = 0.0f;

  EkfYawConfig cfg;
  cfg.enable_periodic_realignment = true;
  cfg.realignment_threshold = 2.0f * (float)M_PI/180.0f; // 2 deg
  cfg.realignment_count_threshold = 2;
  cfg.alignment_timeout_ms = 10;

  float imu_yaw = 0.0f;

  ASSERT_TRUE(ekfYawUpdate(imu_yaw, cfg));

  for(int i=0;i<3;++i){
    robot_theta += 5.0f * (float)M_PI/180.0f;
    advance_millis(20);
    ASSERT_TRUE(ekfYawUpdate(imu_yaw, cfg));
  }

  EXPECT_NEAR(g_yaw_offset, wrapToPi(robot_theta - imu_yaw), 0.05f);
}
