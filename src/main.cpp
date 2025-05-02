#include "main.h"
#include "subsystems.hpp"
#include "lady_brown.hpp"
#include "tasks.hpp"

ez::Drive chassis(
	{6, -5, -4},
	{-3, 2, 1},
	7,
	3.25,
	400);

void initialize()
{
	default_constants();
	chassis.odom_theta_flip(false); // TRUE for blue, FALSE for red
	chassis.opcontrol_curve_buttons_toggle(false);
	chassis.opcontrol_curve_default_set(5.0, 5.0);

	lb.set_brake_mode(MOTOR_BRAKE_HOLD);
	lb_rotation.set_reversed(true);
	lb_rotation.reset_position();
	intake_power = 0;
	intake_running = false;

	ez::as::auton_selector.autons_add({{"Red Negative", red_n},
                                    {"Red Positive", red_p},
                                    {"Blue Negative", blue_n},
                                    {"Blue Positive", blue_p},
                                    {"Red SAWP", red_sawp},
                                    {"Blue SAWP", blue_sawp},
                                    {"Skills", skills},
                                    {"Skills No Lady Brown", skills_no_lb}});
	chassis.initialize();
	ez::as::initialize();
  colour_sensor.set_integration_time(25);
	colour_sensor.set_led_pwm(100);
  
  start_lb_update_task();
  start_colour_rejection_task();
}

void disabled() {}
void competition_initialize() {}

void autonomous()
{
  start_auton_tasks();
  
	chassis.pid_targets_reset();
	chassis.drive_imu_reset();
	chassis.drive_sensor_reset();
	chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

	ez::as::auton_selector.selected_auton_call();
}

void opcontrol()
{
  static LBStage current_lb_stage = LBStage::START;
  lady_brown.set_stage(current_lb_stage);
  lb.set_brake_mode(MOTOR_BRAKE_HOLD);
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  
  int last_intake_power = intake_power;
  intake.move(intake_power);

  while (true)
  {
    chassis.opcontrol_arcade_standard(ez::SPLIT);

    if (master.get_digital_new_press(DIGITAL_L2))
      intake_running = !intake_running;

    intake_power = intake_running ? 127 : 0;

    if (master.get_digital(DIGITAL_L1))
      intake_power = -127;

    if (master.get_digital_new_press(DIGITAL_R1))
    {
      mogo_on = !mogo_on;
      mogo.set_value(mogo_on);
    }
    if (master.get_digital_new_press(DIGITAL_A))
    {
      doinker_on = !doinker_on;
      doinker.set_value(doinker_on);
    }
    if (master.get_digital_new_press(DIGITAL_X))
    {
      if (current_lb_stage == LBStage::PICKUP)
        current_lb_stage = LBStage::SCORE;
      else
        current_lb_stage = LBStage::PICKUP;
      lady_brown.set_stage(current_lb_stage);
    }

    if (master.get_digital_new_press(DIGITAL_UP))
    {
      switch (current_lb_stage)
      {
        case LBStage::START:
          current_lb_stage = LBStage::PICKUP;
          break;
        case LBStage::PICKUP:
          current_lb_stage = LBStage::SCORE;
          break;
        case LBStage::REACH:
          current_lb_stage = LBStage::SCORE;
          break;
        case LBStage::SCORE:
          current_lb_stage = LBStage::END;
          break;
        case LBStage::END:
          break;
      }
      lady_brown.set_stage(current_lb_stage);
    }
    else if (master.get_digital_new_press(DIGITAL_DOWN))
    {
      switch (current_lb_stage)
      {
        case LBStage::START:
          break;
        case LBStage::PICKUP:
          current_lb_stage = LBStage::START;
          break;
        case LBStage::REACH:
          current_lb_stage = LBStage::PICKUP;
          break;
        case LBStage::SCORE:
          current_lb_stage = LBStage::PICKUP;
          break;
        case LBStage::END:
          current_lb_stage = LBStage::SCORE;
          break;
      }
      lady_brown.set_stage(current_lb_stage);
    }
    
    if (intake_power != last_intake_power)
    {
      intake.move(intake_power);
      last_intake_power = intake_power;
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
