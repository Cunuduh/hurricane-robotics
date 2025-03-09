#include "main.h"

void default_constants()
{
  chassis.pid_drive_constants_set(20.0, 0.0, 80.0);  // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0); // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.0, 20.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 300_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
}

void normal_p()
{
  // MAKE SURE TO UNFLIP THETA if red
  // DOINKER WILL NOT WORK VERY WELL FOR RED, USE ALT
  bool alt_autonomous = true;
  activate_lb(320);
  chassis.pid_drive_set(-1.425_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(-30, 64);
  chassis.pid_wait();

  chassis.pid_drive_set(-2.35_ft, 127);

  pros::delay(700);
  activate_sol(true);
  chassis.pid_wait();

  activate_intake(200);

  chassis.pid_drive_set(4_in, 64);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, 64);
  chassis.pid_wait();

  if (alt_autonomous)
  {
    chassis.pid_drive_set(2.5_ft, 127);
    chassis.pid_wait();

    chassis.pid_drive_set(-6_in, 96);
    chassis.pid_wait();

    chassis.pid_turn_set(135, 64);
    chassis.pid_wait();

    activate_sol(false);

    chassis.pid_turn_set(-5, 64);
    chassis.pid_wait();
    
    chassis.pid_drive_set(-1.3_ft, 80);
    
    pros::delay(550);
    activate_sol(true);
    chassis.pid_wait();

    activate_intake(0);
    chassis.pid_drive_set(1.3_ft, 80);
    chassis.pid_wait();
    return;
  }
  chassis.pid_drive_set(2.5_ft, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(-2.5_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(45, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(1.5_ft, 127);
  chassis.pid_wait();

  chassis.pid_swing_set(LEFT_SWING, 90, 64);
  chassis.pid_wait();

  activate_doinker(true);
  pros::delay(500);

  chassis.pid_turn_set(0, 64);
  chassis.pid_wait();

  activate_doinker(false);

  chassis.pid_turn_set(-45, 64);
  chassis.pid_wait();

  chassis.pid_drive_set(2_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(45, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_ft, 127);
  chassis.pid_wait();

  activate_sol(false);
  activate_intake(0);

  chassis.pid_turn_set(135, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(4_ft, 127);
  chassis.pid_wait();
}
void normal_n()
{
  // MAKE SURE TO UNFLIP THETA IF RED
  // CORNER CLEAR WILL NOT WORK VERY WELL FOR RED
  activate_lb(320);
  chassis.pid_drive_set(-1.425_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(30, 64);
  chassis.pid_wait();

  chassis.pid_drive_set(-2.35_ft, 127);

  pros::delay(700);
  activate_sol(true);
  chassis.pid_wait();

  activate_intake(200);

  chassis.pid_drive_set(4_in, 72);
  chassis.pid_wait();

  chassis.pid_turn_set(90, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(2.25_ft, 96);
  chassis.pid_wait();

  chassis.pid_turn_set(190, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(1.3_ft, 80);
  chassis.pid_wait();

  pros::delay(250);

  chassis.pid_drive_set(-1.3_ft, 80);
  chassis.pid_wait();

  chassis.pid_turn_set(0, 64);
  chassis.pid_wait();

  chassis.pid_drive_set(3.05_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(82.5, 64);
  chassis.pid_wait();

  chassis.pid_drive_set(4_in, 64);
  chassis.pid_wait();

  activate_doinker(true);
  pros::delay(500);

  if (!chassis.odom_theta_direction_get()) // IF RED
  {
    chassis.pid_turn_set(0, 80);
  }
  else
  {
    chassis.pid_turn_set(270, 80);
  }
  chassis.pid_wait();

  activate_doinker(false);

  chassis.pid_turn_set(225, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(4_ft, 80);

  pros::delay(1200);
  activate_intake(0);

  chassis.pid_wait();

  pros::delay(5000);
}
void skills()
{
  // MAKE SURE TO UNFLIP THETA
  activate_lb(320);
  intake.move_velocity(200);
  pros::delay(2000);
  intake.move_velocity(0);

  chassis.pid_drive_set(1.25_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(90, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(-2.45_ft, 127);

  pros::delay(500);
  activate_sol(true);
  chassis.pid_wait();

  intake_power = 200;
  intake.move_velocity(200);

  chassis.pid_turn_set(0, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(2_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(2_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(-25, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(1.6_ft, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.6_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(3.25_ft, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.25_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(270, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(1.25_ft, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.25_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(40, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.75_ft, 127);

  activate_sol(false);
  activate_intake(0);

  chassis.pid_wait();

  chassis.pid_drive_set(1.75_ft, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(270, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(-5_ft, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_ft, 127);
  pros::delay(500);
  activate_sol(true);
  chassis.pid_wait();

  activate_intake(200);
}