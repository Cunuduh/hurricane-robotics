#include "main.h"

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

  lb_pid.exit_condition_set(80, 1, 250, 3, 250, 250);
}
void normal_n() {}
void normal_p() {}
void skills()
{
  // MAKE SURE TO UNFLIP THETA
  // ptp = point-to-point, straight line
  // pp = pure pursuit, curved
  // boomerang = curved, used for posing at a specific heading
  chassis.odom_xyt_set(-5.25_ft, 0_ft, 90_deg);
  chassis.pid_odom_behavior_set(shortest);
  activate_intake(200, 1000);
  chassis.pid_odom_set(1.0_ft, 110);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
  chassis.pid_odom_set({{-4.0_ft, 2.0_ft}, rev, 80});
  chassis.pid_wait();

  activate_mogo(true);
  activate_intake(200);

  chassis.pid_odom_set({
    {{-2.0_ft, 2.0_ft}, fwd, 110},
    {{2.75_ft, 4.0_ft, 90_deg}, fwd, 110},
    {{0.0_ft, 3.5_ft, 0_deg}, fwd, 110}
  });
  chassis.pid_wait_until_index(1);
  set_lb_stage(LBStage::PICKUP);
  wait_for_ring();
  set_lb_stage(LBStage::REACH);
  chassis.pid_wait();

  chassis.pid_odom_set(2.0_ft, 110);
  chassis.pid_wait();

  set_lb_stage(LBStage::SCORE);
  lb_pid_wait();

  chassis.pid_odom_set(-2.0_ft, 110);
  chassis.pid_wait();

  set_lb_stage(LBStage::START);

  chassis.pid_turn_set({-2.0_ft, 4.0_ft}, fwd, 110);
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{-2.0_ft, 4.0_ft, 270_deg}, fwd, 110},
    {{-5.75_ft, 4.0_ft}, fwd, 80},
    {{-3.5_ft, 5.5_ft}, fwd, 80},
    {{-5.75_ft, -5.75_ft, 135_deg}, rev, 110}
  });
  chassis.pid_wait_until_index(3);
  activate_mogo(false);
  chassis.pid_wait();

  chassis.pid_odom_set({{-4.0_ft, 4.0_ft}, fwd, 110});
  chassis.pid_wait();

  chassis.pid_odom_set({{-4.0_ft, -2.0_ft}, rev, 80});
  chassis.pid_wait();

  activate_mogo(true);

  chassis.pid_odom_set({
    {{-2.0_ft, -2.0_ft, 90_deg}, fwd, 110},
    {{2.75_ft, -4.0_ft}, fwd, 110},
    {{0.0_ft, -3.5_ft, 180_deg}, fwd, 110},
  });
  chassis.pid_wait_until_index(1);
  set_lb_stage(LBStage::PICKUP);
  wait_for_ring();
  set_lb_stage(LBStage::REACH);
  chassis.pid_wait();

  chassis.pid_odom_set(2.0_ft, 110);
  chassis.pid_wait();

  set_lb_stage(LBStage::SCORE);
  lb_pid_wait();

  chassis.pid_odom_set(-2.0_ft, 110);
  chassis.pid_wait();

  set_lb_stage(LBStage::START);

  chassis.pid_turn_set({-2.0_ft, -4.0_ft}, fwd, 110);
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{-2.0_ft, -4.0_ft, 270_deg}, fwd, 110},
    {{-5.75_ft, -4.0_ft}, fwd, 80},
    {{-3.5_ft, -5.5_ft}, fwd, 80},
    {{-5.75_ft, 5.75_ft, 45_deg}, rev, 110}
  });
  chassis.pid_wait_until_index(3);
  activate_mogo(false);
  chassis.pid_wait();

  chassis.pid_odom_set({{2.0_ft, -2.0_ft}, fwd, 110});
  chassis.pid_wait();

  set_lb_stage(LBStage::PICKUP);
  wait_for_ring();
  set_lb_stage(LBStage::REACH);

  chassis.pid_odom_set({{5.0_ft, -2.0_ft}, rev, 110});
  chassis.pid_wait();

  activate_mogo(true);

  chassis.pid_odom_set({{5.0_ft, -4.0_ft, 180_deg}, fwd, 110});
  chassis.pid_wait();

  pros::delay(750);
  activate_intake(0);

  chassis.pid_odom_set({{4.5_ft, -5.0_ft}, fwd, 110});
  activate_doinker(true);
  pros::delay(250);
  chassis.pid_wait();

  chassis.pid_turn_set({5.75_ft, -6.0_ft}, fwd, 110);
  chassis.pid_wait();

  chassis.pid_odom_set(1.0_ft, 110);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();

  activate_doinker(false);

  chassis.pid_turn_set(315_deg, 110);
  chassis.pid_wait();

  activate_mogo(false);

  chassis.pid_odom_set(-1.0_ft, 110);
  chassis.pid_wait();

  chassis.pid_odom_set(1.0_ft, 110);
  chassis.pid_wait();

  chassis.pid_odom_set({{4.0_ft, 0.0_ft}, rev, 110});
  chassis.pid_wait();

  activate_mogo(true);

  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait();

  chassis.pid_odom_set(1.0_ft, 110);
  chassis.pid_wait();

  set_lb_stage(LBStage::END);
  lb_pid_wait();

  chassis.pid_odom_set(-1.0_ft, 110);
  chassis.pid_wait();

  set_lb_stage(LBStage::START);
  activate_intake(200);

  chassis.pid_odom_set({
    {{2.0_ft, 2.0_ft}, fwd, 110},
    {{-0.5_ft, -0.5_ft}, fwd, 110},
    {{4.0_ft, 4.0_ft}, fwd, 110},
    {{5.25_ft, 4.0_ft}, fwd, 55},
    {{3.75_ft, 5.25_ft}, fwd, 55},
  });
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait();

  activate_doinker(true);

  chassis.pid_odom_set(2.0_ft, 110);
  chassis.pid_wait();
  
  chassis.pid_turn_set(315_deg, 110, ccw);
  chassis.pid_wait();
  
  activate_doinker(false);

  chassis.pid_turn_set(225_deg, 110);
  chassis.pid_wait();

  activate_mogo(false);
  chassis.pid_odom_set(-1.0_ft, 110);
  chassis.pid_wait();
  chassis.pid_odom_set(6.0_ft, 110);
  chassis.pid_wait();
}
