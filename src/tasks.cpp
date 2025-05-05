#include "main.h"
#include "tasks.hpp"
#include "lady_brown.hpp"
#include "subsystems.hpp"

void start_colour_rejection_task() {
  static pros::Task colour_rejection_task_handle{[] {
    uint32_t notify_val = 0;
    while (true)
    {
      // block forever, until a notification and value is received
      notify_val = pros::Task::notify_take(true, TIMEOUT_MAX);
      // now do a timed wait for the switch
      uint32_t mark = pros::millis();
      while (!limit_switch.get_value() &&
             (pros::millis() - mark < 250))
      {
        pros::Task::delay_until(&mark, 1);
      }
      if (limit_switch.get_value())
      {
        while(limit_switch.get_value() && (pros::millis() - mark < 100)) {
          pros::Task::delay_until(&mark, 1);
        }
        intake_conveyor.move(0);
        pros::Task::delay_until(&mark, 200);
        intake_conveyor.move(notify_val);// resume at saved speed
      }
      // loops back, blocks again
    }
  }, "clr_reject"};

  static pros::Task colour_detection_task_handle{[]
  {
    uint32_t last = pros::millis();
    while (true)
    {
      pros::Task::delay_until(&last, 10);
      if (intake_power == 127)
      {
        auto detected = detect_colour();
        if (detected != Colour::NONE && detected != team_colour)
        {
          // send current speed as notification value
          colour_rejection_task_handle.notify_ext(
            intake_power,
            pros::E_NOTIFY_ACTION_OWRITE,
            nullptr
          );
        }
      }
    }
  }, "clr_detect"};
}

void start_lb_update_task()
{
  static pros::Task lb_task_handle{[]
  {
    while (true)
    {
      lady_brown.update();
      pros::delay(ez::util::DELAY_TIME);
    }
  }, "lb_update"};
}

void start_auton_tasks()
{
  static pros::Task intake_task_handle{[]
  {
    pros::delay(2500);
    while (true)
    {
      if (!pros::competition::is_autonomous()) return;
      if (lady_brown.get_stage() != LBStage::PICKUP && is_intake_stalled(intake))
      {
        intake_conveyor.move(-127);
        pros::delay(100);
        intake_conveyor.move(intake_power);
      }
      pros::delay(100);
    }
  }, "intake_unjam"};
}
