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
  chassis.odom_xyt_set(-5.25_ft, 2.0_ft, -90_deg);
  chassis.pid_turn_behavior_set(shortest);
  chassis.pid_odom_set({
    {{-2.25_ft, 2.0_ft}, rev, 80},
    {{-2.0_ft, 2.0_ft}, rev, 55}
  });
  chassis.pid_wait_until_index(0);
  activate_mogo(true);
  activate_intake(127);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();
  auto ring_pos = chassis.odom_x_direction_get() ? 3.5_ft : 4.5_ft; // RED unflipped, BLUE flipped

  chassis.pid_odom_set(ring_pos - 2.0_ft, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait();
  activate_doinker(true);
  pros::delay(750);
  chassis.pid_odom_set({
    {{-0.5_ft, ring_pos}, fwd, 110},
    {{-2.0_ft, ring_pos}, rev, 110}
  });
  chassis.pid_wait();

  chassis.pid_turn_set(135_deg, 110);
  chassis.pid_wait();
  activate_doinker(false);
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set(2.0_ft, 55);
  chassis.pid_wait();

  if (chassis.odom_x_direction_get()) // BLUE
  {
    chassis.pid_odom_set({
      {{-5.0_ft, 4.5_ft, 0_deg}, fwd, 110},
      {{-5.0_ft, 5.5_ft}, fwd, 80},
      {{-5.0_ft, 4.5_ft}, rev, 110}
    });
    chassis.pid_wait_until_index(0);
    activate_doinker(true);
    chassis.pid_wait();
    chassis.pid_turn_set(90_deg, 110);
    chassis.pid_wait();
    activate_doinker(false);
    chassis.pid_turn_set(60_deg, 110);
    chassis.pid_wait();
    chassis.pid_odom_set(2.0_ft, 55);
  }
  else
  {
    chassis.pid_odom_set({
      {{-4.5_ft, 5.0_ft, -90_deg}, fwd, 110},
      {{-5.5_ft, 5.0_ft}, fwd, 80},
      {{-4.5_ft, 5.0_ft}, rev, 110}
    });
    chassis.pid_wait_until_index(0);
    activate_doinker(true);
    chassis.pid_wait();
    chassis.pid_turn_set(-180_deg, 110);
    chassis.pid_wait();
    activate_doinker(false);
    chassis.pid_turn_set(-165_deg, 110);
    chassis.pid_wait();
    chassis.pid_odom_set(2.0_ft, 55);
  }
}
void positive()
{

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
void red_sawp() {}
void blue_sawp() {}
void skills()
{
  // MAKE SURE TO UNFLIP THETA
  // ptp = point-to-point, straight line
  // pp = pure pursuit, curved
  // boomerang = curved, used for posing at a specific heading
  chassis.odom_xyt_set(-5.25_ft, 0_ft, 90_deg);
  chassis.pid_turn_behavior_set(shortest);
  activate_intake(127, 1000);
  chassis.pid_odom_set(1.0_ft, 110);
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
    {{0.0_ft, 3.75_ft}, fwd, 110},
    {{2.0_ft, 4.0_ft}, fwd, 80},
    {{2.75_ft, 4.5_ft}, fwd, 55},
    {{0.0_ft, 3.75_ft}, fwd, 80}
  });
  chassis.pid_wait_until_index(2);
  lady_brown.set_stage(LBStage::PICKUP);
  lady_brown.wait_until_settled();
  push_into_lb();
  lady_brown.set_stage(LBStage::REACH);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();

  activate_intake(127);

  chassis.pid_odom_set(1.5_ft, 55);
  chassis.pid_wait();

  lady_brown.set_stage(LBStage::SCORE);
  lady_brown.wait_until_settled();

  chassis.pid_odom_set(-1.5_ft, 110);
  chassis.pid_wait();

  lady_brown.set_stage(LBStage::START);

  chassis.pid_turn_set({-2.0_ft, 4.0_ft}, fwd, 110);
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{-2.0_ft, 4.0_ft, -90_deg}, fwd, 110},
    {{-4.0_ft, 4.0_ft}, fwd, 80},
    {{-5.5_ft, 4.0_ft}, fwd, 30},
    {{-3.5_ft, 5.5_ft}, fwd, 55},
    {{-5.5_ft, 5.5_ft, 135_deg}, rev, 80}
  });
  chassis.pid_wait_until_index(4);
  activate_mogo(false);
  activate_intake(0);
  chassis.pid_wait();
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
    {{0.0_ft, -3.75_ft}, fwd, 110},
    {{2.0_ft, -4.0_ft}, fwd, 80},
    {{2.75_ft, -4.5_ft}, fwd, 55},
    {{0.0_ft, -3.75_ft}, fwd, 80},
  });
  chassis.pid_wait_until_index(2);
  lady_brown.set_stage(LBStage::PICKUP);
  lady_brown.wait_until_settled();
  push_into_lb();
  lady_brown.set_stage(LBStage::REACH);
  chassis.pid_wait();
  activate_intake(0);

  chassis.pid_turn_set(180_deg, 110);

  chassis.pid_odom_set(1.5_ft, 55);
  chassis.pid_wait();

  activate_intake(127);

  lady_brown.set_stage(LBStage::SCORE);
  lady_brown.wait_until_settled();

  chassis.pid_odom_set(-1.5_ft, 110);
  chassis.pid_wait();

  lady_brown.set_stage(LBStage::START);

  chassis.pid_turn_set({-2.0_ft, -4.0_ft}, fwd, 110);
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{-2.0_ft, -4.0_ft}, fwd, 110},
    {{-4.0_ft, -4.0_ft}, fwd, 80},
    {{-5.5_ft, -4.0_ft}, fwd, 30},
    {{-3.5_ft, -5.5_ft}, fwd, 55},
    {{-5.5_ft, -5.5_ft, 45_deg}, rev, 110}
  });
  chassis.pid_wait_until_index(4);
  activate_mogo(false);
  activate_intake(0);
  chassis.pid_wait();
  // PART 3
  activate_intake(127);
  chassis.pid_odom_set({{2.0_ft, -2.0_ft}, fwd, 110});
  chassis.pid_wait();

  lady_brown.set_stage(LBStage::PICKUP);
  lady_brown.wait_until_settled();
  push_into_lb();
  lady_brown.set_stage(LBStage::REACH);

  chassis.pid_odom_set({
    {{4.0_ft, 0.0_ft}, rev, 110},
    {{5.0_ft, 1.0_ft}, rev, 55},
    {{4.0_ft, 0.0_ft}, fwd, 55},
  });
  chassis.pid_wait_until_index(0);
  activate_mogo(true);
  activate_intake(127);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait();

  lady_brown.set_stage(LBStage::END);
  lady_brown.wait_until_settled();
  chassis.pid_odom_set(-1.5_ft, 110);
  chassis.pid_wait();
  lady_brown.set_stage(LBStage::START);
  chassis.pid_odom_set({
    {{2.0_ft, 2.0_ft}, fwd, 110},
    {{-0.5_ft, -0.5_ft}, fwd, 80},
    {{4.0_ft, 4.0_ft}, fwd, 110},
    {{5.0_ft, 4.0_ft}, fwd, 55},
    {{4.0_ft, 5.0_ft}, fwd, 55},
    {{5.5_ft, 5.5_ft, -135_deg}, rev, 110},
    {{4.0_ft, 0.0_ft}, fwd, 110},
    {{5.0_ft, -2.0_ft}, fwd, 110},
    {{5.75_ft, -5.75_ft}, fwd, 110}
  });
  chassis.pid_wait_until_index(5);
  activate_mogo(false);
  chassis.pid_wait();
}
void skills_no_lb()
{
  // MAKE SURE TO UNFLIP THETA
  // ptp = point-to-point, straight line
  // pp = pure pursuit, curved
  // boomerang = curved, used for posing at a specific heading
  chassis.odom_xyt_set(-5.25_ft, 0_ft, 90_deg);
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
    {{0.0_ft, 5.25_ft}, fwd, 55},
    {{0.0_ft, 4.0_ft}, rev, 80}
  });
  chassis.pid_wait();

  chassis.pid_turn_set({-2.0_ft, 4.0_ft}, fwd, 110);
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{-2.0_ft, 4.0_ft}, fwd, 110},
    {{-4.0_ft, 4.0_ft}, fwd, 80},
    {{-5.25_ft, 4.0_ft}, fwd, 30},
    {{-3.75_ft, 5.25_ft}, fwd, 80},
    {{-5.25_ft, 5.25_ft, 135_deg}, rev, 80}
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
    {{0.0_ft, -5.25_ft}, fwd, 55},
    {{0.0_ft, -4.0_ft}, rev, 80}
  });
  chassis.pid_wait();

  chassis.pid_turn_set({-2.0_ft, -4.0_ft}, fwd, 110);
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{-2.0_ft, -4.0_ft}, fwd, 110},
    {{-4.0_ft, -4.0_ft}, fwd, 80},
    {{-5.25_ft, -4.0_ft}, fwd, 30},
    {{-3.75_ft, -5.25_ft}, fwd, 80},
    {{-5.25_ft, -5.25_ft, 45_deg}, rev, 110}
  });
  chassis.pid_wait();
  activate_mogo(false);
  activate_intake(0);
  // PART 3
  activate_intake(127);
  chassis.pid_odom_set({
    {{2.0_ft, -2.0_ft}, fwd, 110},
    {{2.25_ft, -1.75_ft}, fwd, 55}
  });
  chassis.pid_wait();
  pros::delay(50);
  activate_intake(0);
  chassis.pid_odom_set({{4.0_ft, -1.0_ft}, fwd, 110});
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set(1.0_ft, 30);
  chassis.pid_wait();
  chassis.pid_odom_set(-1.0_ft, 55);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{4.0_ft, 0.0_ft}, rev, 80},
    {{5.25_ft, 0.0_ft}, rev, 30}
  });
  chassis.pid_wait();

  activate_intake(127);
  pros::delay(500);
  chassis.pid_odom_set({{4.0_ft, 0.0_ft}, fwd, 110});
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{4.0_ft, 1.0_ft}, rev, 110},
    {{3.75_ft, 1.75_ft}, rev, 55}
  });
  chassis.pid_wait_until_index(0);
  activate_mogo(true);
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{2.0_ft, 2.0_ft}, fwd, 110},
    {{-0.5_ft, -0.5_ft}, fwd, 80},
    {{2.0_ft, -2.0_ft}, fwd, 110},
    {{2.0_ft, -4.0_ft}, fwd, 110},
    {{4.0_ft, 0.0_ft}, fwd, 110},
    {{2.0_ft, 4.0_ft}, fwd, 110},
    {{4.0_ft, 4.0_ft}, fwd, 55},
    {{5.0_ft, 4.0_ft}, fwd, 55},
    {{5.25_ft, 5.25_ft, -135_deg}, rev, 110},
    {{4.0_ft, 0.0_ft}, fwd, 110},
    {{5.0_ft, -2.0_ft}, fwd, 110},
    {{5.25_ft, -5.25_ft}, fwd, 110}
  });
  chassis.pid_wait_until_index(8);
  activate_mogo(false);
  chassis.pid_wait();
}
