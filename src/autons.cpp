#include "main.h"

void default_constants()
{
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
}

void normal_p() {
  activate_lb(300);
  chassis.pid_drive_set(-1.425_ft, 64);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(30, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(-2.5_ft, 64); 
  pros::delay(500);
  activate_sol(true);
  pros::delay(500);
  activate_intake(200);

  chassis.pid_turn_relative_set(-120, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(2.5_ft, 64);
  chassis.pid_wait();

  pros::delay(5000);
  intake_power = 0;
  intake.move_velocity(0);
}

void normal_n() {
  activate_lb(300);
  chassis.pid_drive_set(-1.425_ft, 64);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(30, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(-2.75_ft, 64);

  pros::delay(500);
  activate_sol(true);
  chassis.pid_wait();

  activate_intake(200);

  chassis.pid_turn_relative_set(60, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(2_ft, 64);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(75, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(1.125_ft, 32);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.125_ft, 32);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(30, 48);
  chassis.pid_wait();

  chassis.pid_drive_set(1.125_ft, 64);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.125_ft, 64);
  chassis.pid_wait();

  pros::delay(3000);
  activate_intake(0);
}
