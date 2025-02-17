#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

inline pros::Optical colour_sensor(19);
inline pros::Rotation lb_rotation(20);
inline pros::MotorGroup intake({8, -9}, pros::v5::MotorGears::green);
inline pros::Motor intake_conveyor(8, pros::v5::MotorGears::green);
inline pros::Motor lb(10, pros::v5::MotorGears::green);
inline pros::ADIDigitalOut solenoid('A');
inline pros::ADIDigitalOut doinker('B');
inline std::atomic<int32_t> intake_power{0};
inline bool intake_running = false;
inline bool solenoid_on = false;
inline bool doinker_on = false;