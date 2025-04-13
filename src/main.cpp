#include "main.h"
ez::Drive chassis(
	{6, -5, -4},
	{-3, 2, 1},
	7,
	3.25,
	400);

const double START_PICKUP_THRESHOLD = 16.0;   // Between START (0) and PICKUP (32)
const double PICKUP_REACH_THRESHOLD = 61.0;   // Between PICKUP (32) and REACH (90)
const double REACH_SCORE_THRESHOLD = 110.0;   // Between REACH (60) and SCORE (160)
const double SCORE_END_THRESHOLD = 215.0;     // Between SCORE (160) and END (270)

void initialize()
{
	default_constants();
	chassis.odom_theta_flip(false); // TRUE for blue, FALSE for red
	chassis.opcontrol_curve_buttons_toggle(false);
	chassis.opcontrol_curve_default_set(5.0, 5.0);

	lb.set_brake_mode(MOTOR_BRAKE_HOLD);
  lb_rotation.set_reversed(true);
  set_lb_stage(LBStage::START);
	intake_power = 0;
	intake_running = false;

	ez::as::auton_selector.autons_add({{"Normal Negative", normal_n},
																		 {"Normal Positive", normal_p},
                                     {"Skills", skills}});
	chassis.initialize();
	ez::as::initialize();
	colour_sensor.set_led_pwm(100);
  pros::Task lb_task([] {
    while (true)
    {
      move_lb(ez::util::clamp(lb_pid.compute(get_lb_angle()), 127, -127));
      pros::delay(ez::util::DELAY_TIME);
    }
  });
	pros::Task colour_rejection_task{[&]
	{
		while (true)
		{
			//if (!pros::competition::is_autonomous())
			//	return;
			if (intake_power != 0)
			{
				Colour detected = detect_colour();
				if (detected != Colour::NONE && detected != team_colour)
				{
					colour_rejection_active = true;
					pros::delay(215);
					intake.move_velocity(0);
					pros::delay(200);
					intake.move_velocity(intake_power);
					colour_rejection_active = false;
				}
			}
			pros::delay(30);
		}
	}};
}
void disabled() {}
void competition_initialize() {}
void autonomous()
{
	pros::Task intake_task{[&]
	{
		while (true)
		{
			if (!pros::competition::is_autonomous())
				return;
			if (get_lb_angle() > 40 && is_intake_stalled(intake))
			{
				attempt_unjam();
				pros::delay(100);
			}
			pros::delay(100);
		}
	}};
	colour_rejection_active = false;
	chassis.pid_targets_reset();
	chassis.drive_imu_reset();
	chassis.drive_sensor_reset();
	chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

	ez::as::auton_selector.selected_auton_call();
}

void opcontrol()
{
  static LBStage current_lb_stage = LBStage::START;
  set_lb_stage(current_lb_stage);
	lb.set_brake_mode(MOTOR_BRAKE_HOLD);
	chassis.drive_brake_set(MOTOR_BRAKE_COAST);
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
      set_lb_stage(current_lb_stage);
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
      set_lb_stage(current_lb_stage);
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
      set_lb_stage(current_lb_stage);
    }
		if (!colour_rejection_active) intake.move(intake_power);
		pros::delay(ez::util::DELAY_TIME);
	}
}
