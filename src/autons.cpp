#include "main.h"
#include "lady_brown.hpp"
#include "subsystems.hpp"

void default_constants()
{
  chassis.pid_drive_constants_set(20.0, 0.0, 80.0);  // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0); // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.0, 20.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);
  chassis.pid_odom_angular_constants_set(8.0, 0.0, 80.0);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(4.0, 0.0, 60.0);  // Angular control for boomerang motions

  chassis.pid_turn_exit_condition_set(50_ms, 3_deg, 200_ms, 7_deg, 300_ms, 400_ms);
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 200_ms, 3_in, 400_ms, 400_ms);
  chassis.pid_swing_exit_condition_set(50_ms, 3_deg, 200_ms, 7_deg, 400_ms, 400_ms);
}
void negative()
{
  chassis.odom_xyt_set(-5.25_ft, 4.0_ft, -90_deg);
  chassis.pid_turn_behavior_set(shortest);
  chassis.pid_odom_set(-1.0_ft, 80);
  chassis.pid_odom_set({
    {{-2.0_ft, 2.0_ft}, rev, 110},
    {{-1.75_ft, 1.75_ft}, rev, 55}
  });
  chassis.pid_wait_until_index(0);
  activate_mogo(true);
  activate_intake(127);
  chassis.pid_wait();

  chassis.pid_odom_set(0.5_ft, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();

  chassis.pid_odom_set(2.5_ft, 110);
  chassis.pid_wait();
  if (chassis.odom_x_direction_get()) // BLUE
  {
    chassis.pid_odom_set(-1.0_ft, 55);
    chassis.pid_wait();
    chassis.pid_turn_set(80_deg, 110);
    chassis.pid_wait();
  }
  else // RED
  {
    chassis.pid_turn_set(97_deg, 110);
    chassis.pid_wait();
  }
  chassis.pid_odom_set(-0.5_ft, 55);
  chassis.pid_wait();
  activate_doinker(true);
  pros::delay(750);
  chassis.pid_odom_set(1.75_ft, 80);
  chassis.pid_wait();
  chassis.pid_odom_set(-1.75_ft, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(135_deg, 110);
  chassis.pid_wait();
  activate_doinker(false);
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set(2.0_ft, 55); // get grabbed ring
  chassis.pid_wait();
  if (chassis.odom_x_direction_get()) // BLUE
  {
    chassis.pid_odom_set({{-4.33_ft, 1.25_ft, 180_deg}, fwd, 110});
    chassis.pid_wait();
  }
  else
  {
    chassis.pid_odom_set({{-3.33_ft, 1.25_ft, 180_deg}, fwd, 110});
    chassis.pid_wait();
  }
  activate_doinker(true);
  pros::delay(500);
  chassis.pid_turn_set(135_deg, 110);
  chassis.pid_wait();
  activate_doinker(false);
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set(1.5_ft, 80);
  chassis.pid_wait();
}
void positive()
{
  chassis.odom_xyt_set(-5.25_ft, -2.0_ft, -90_deg); 
  chassis.pid_turn_behavior_set(shortest);
  chassis.pid_odom_set({
    {{-2.0_ft, -2.0_ft}, rev, 80},
    {{-1.5_ft, -2.0_ft}, rev, 55}
  });
  chassis.pid_wait_until_index(0);
  activate_mogo(true);
  activate_intake(127);
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_turn_set(-135_deg, 110);
  chassis.pid_wait();
  activate_mogo(false);
  chassis.pid_odom_set({{-2.0_ft, -4.0_ft}, fwd, 110});
  chassis.pid_wait();
  activate_intake(0);
  chassis.pid_turn_set(-90_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{-0.5_ft, -4.0_ft}, rev, 110},
    {{0.0_ft, -4.0_ft}, rev, 55}
  });
  chassis.pid_wait_until_index(0);
  activate_mogo(true);
  activate_intake(127, 1000);
}
void red_n()
{
  negative();
}
void blue_n()
{
  chassis.odom_x_flip(true);
  chassis.odom_theta_flip(true);
  negative();
}
void red_p()
{
  positive();
}
void blue_p()
{
  chassis.odom_x_flip(true);
  chassis.odom_theta_flip(true);
  positive();
}
void skills()
{
  // MAKE SURE TO UNFLIP THETA
  // ptp = point-to-point, straight line
  // pp = pure pursuit, curved
  // boomerang = curved, used for posing at a specific heading
  chassis.odom_xyt_set(-5.167_ft, 0_ft, 90_deg);
  chassis.pid_turn_behavior_set(shortest);
  activate_intake(127, 1000);
  chassis.pid_odom_set(1.25_ft, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{-4.0_ft, 2.0_ft}, rev, 110},
    {{-4.0_ft, 2.25_ft}, rev, 55},
  });

  chassis.pid_wait_until_index(0);
  activate_mogo(true);
  activate_intake(127);
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{-2.0_ft, 2.0_ft}, fwd, 110},
    {{0.0_ft, 3.5_ft}, fwd, 110}
  });
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{0.0_ft, 5.5_ft}, fwd, 55},
    {{0.0_ft, 4.0_ft}, rev, 80}
  });
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{-2.0_ft, 4.0_ft}, fwd, 110},
    {{-4.0_ft, 4.0_ft}, fwd, 55},
    {{-6.0_ft, 4.0_ft}, fwd, 30},
    {{-4.125_ft, 3.5_ft}, rev, 55},
  });
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{-4.125_ft, 5.5_ft}, fwd, 55},
    {{-5.25_ft, 5.25_ft, 135_deg}, rev, 70}
  });
  chassis.pid_wait();
  activate_mogo(false);
  activate_intake(0);
  // PART 2
  chassis.pid_odom_set({{-4.0_ft, 4.0_ft}, fwd, 110});
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{-4.25_ft, -1.75_ft}, rev, 110},
    {{-4.25_ft, -2.0_ft}, rev, 55},
  });
  chassis.pid_wait_until_index(0);
  activate_mogo(true);
  activate_intake(127);
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{-2.0_ft, -2.0_ft}, fwd, 110},
    {{0.0_ft, -3.5_ft}, fwd, 110}
  });
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{0.0_ft, -5.5_ft}, fwd, 55},
    {{0.0_ft, -4.0_ft}, rev, 80}
  });
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{-2.0_ft, -4.0_ft}, fwd, 110},
    {{-4.0_ft, -4.0_ft}, fwd, 55},
    {{-6.0_ft, -4.0_ft}, fwd, 30},
    {{-4.125_ft, -3.5_ft}, rev, 55},
  });
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{-4.125_ft, -5.5_ft}, fwd, 55},
    {{-5.25_ft, -5.25_ft, 45_deg}, rev, 70}
  });
  chassis.pid_wait();
  activate_mogo(false);
  // PART 3
  chassis.pid_odom_set({
    {{2.0_ft, -2.125_ft}, fwd, 110},
    {{2.125_ft, -2.0_ft}, fwd, 80}
  });
  chassis.pid_wait();
  activate_intake(0);
  chassis.pid_odom_set({{3.5_ft, -2.0_ft}, fwd, 80});
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, 80);
  chassis.pid_wait();
  chassis.pid_odom_set(3.0_ft, 55); // SHOVE THIRD MOGO TO THE SIDE
  chassis.pid_wait();
  chassis.pid_odom_set(-1.0_ft, 55);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set({{5.25_ft, 0.0_ft}, rev, 55});
  chassis.pid_wait();

  activate_intake(127, 1000);
  chassis.pid_odom_set({{4.0_ft, 0.0_ft}, fwd, 110});
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set(-1.25_ft, 80);
  chassis.pid_wait_quick();
  chassis.pid_odom_set(-0.5_ft, 55);
  chassis.pid_wait();
  activate_mogo(true);
  activate_intake(127);
  pros::delay(50);
  chassis.pid_odom_set({
    {{2.0_ft, 2.0_ft}, fwd, 110},
    {{0.0_ft, 0.0_ft}, fwd, 80},
    {{2.0_ft, -2.0_ft}, fwd, 110},
    {{2.0_ft, -4.5_ft}, fwd, 55},
    {{4.25_ft, -4.0_ft}, fwd, 80},
  });
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{3.5_ft, 0.0_ft}, fwd, 127},
  });
  chassis.pid_wait_quick();
  chassis.pid_odom_set({
    {{2.0_ft, 4.0_ft}, fwd, 110},
    {{4.0_ft, 3.875_ft}, fwd, 55},
  });
  chassis.pid_wait();
  activate_doinker(true);
  pros::delay(250);
  chassis.pid_turn_set(0_deg, 110, ccw);
  chassis.pid_wait_quick();
  activate_intake(0);
  chassis.pid_odom_set({{5.25_ft, 5.25_ft, -135_deg}, rev, 110});
  chassis.pid_wait_quick();
  activate_doinker(false);
  activate_mogo(false);
  chassis.pid_odom_set({
    {{3.5_ft, 0.0_ft}, fwd, 127},
    {{5.0_ft, -2.0_ft}, fwd, 127},
    {{5.25_ft, -5.25_ft}, fwd, 127}
  });
  chassis.pid_wait_quick();
  lady_brown.set_stage(LBStage::END);
  chassis.pid_odom_set({{1.0_ft, -1.0_ft}, rev, 127}); // HANG
  chassis.pid_wait();
}
