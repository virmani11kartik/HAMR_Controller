#include <gtest/gtest.h>
#include "Arduino.h"
#include "esp32-hal.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include "WiFi.h"
#include "WiFiUdp.h"

#include "odometry.h"


static void reset_odom(){
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
}

TEST(Odometry, StraightLine){
  reset_odom();
  initOdometry();

  for(int k=0;k<10;++k){
    advance_millis(100);   // 10 Hz
    ticksL += 100;
    ticksR += 100;
    updateOdometry();
  }

  extern float robot_x, robot_y, robot_theta;
  EXPECT_GT(robot_x, 0.0f);
  EXPECT_NEAR(robot_y, 0.0f, 1e-3f);
  EXPECT_NEAR(robot_theta, 0.0f, 1e-3f);
}

TEST(Odometry, PureRotation){
  reset_odom();
  initOdometry();

  for(int k=0;k<10;++k){
    advance_millis(100);
    ticksL += 100;
    ticksR -= 100;
    updateOdometry();
  }

  extern float robot_theta;
  EXPECT_TRUE(std::fabs(robot_theta) > 0.01f);
}

TEST(Odometry, CovarianceGrowth){
  reset_odom();
  initOdometry();

  extern float covariance[6];
  float c0[6]; for(int i=0;i<6;++i) c0[i]=covariance[i];

  for(int k=0;k<5;++k){
    advance_millis(100);
    ticksL += 50;
    ticksR += 50;
    updateOdometry();
  }

  EXPECT_GT(covariance[0], c0[0]); // var x
  EXPECT_GT(covariance[3], c0[3]); // var y
}
