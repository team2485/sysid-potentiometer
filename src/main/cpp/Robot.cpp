// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ctre/Phoenix.h"

#include "math.h"

#include <string>

#include <fmt/core.h>

#include <frc/AnalogPotentiometer.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <wpi/json.h>

#include <iostream>

#include <fstream>

#include <wpi/fs.h>

#include <stdio.h>

#include <Robot.h>

using namespace std;

using namespace frc;

Robot::Robot() : frc::TimedRobot(5_ms){

}

void Robot::RobotInit() {

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_data.clear();
  m_startTime = frc::Timer::GetFPGATimestamp().value();
  m_data.push_back(m_startTime);
}

void Robot::AutonomousPeriodic() {
  m_lastPosition = m_pot.Get();
  
  if (m_testType == "dynamic") {
    if (m_direction == "forward") {
      m_talon.SetVoltage((units::volt_t) m_stepVoltage);
    }
    else {
      m_talon.SetVoltage((units::volt_t) -m_stepVoltage);
    }
  }
  else {
    if(m_direction == "forward") {
      m_talon.SetVoltage((units::volt_t)  m_rampRate * (frc::Timer::GetFPGATimestamp().value() - m_startTime));
    }
    else {
      m_talon.SetVoltage((units::volt_t) - m_rampRate * (frc::Timer::GetFPGATimestamp().value() - m_startTime));
    }
  }

  double m_position = m_pot.Get();
  double m_potVelocity = (m_position - m_lastPosition) / 0.005; 
  
    m_data.insert( m_data.end(), {m_talon.GetMotorOutputVoltage(), m_position, m_potVelocity});

  m_lastPosition = m_pot.Get();

}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
    m_talon.SetVoltage((units::volt_t) 0);
  
  if(m_data.size() > 0) {
    std::string type = m_testType == "dynamic" ? "fast" : "slow";
    std::string direction = m_direction;
    std::string test = fmt::format("\"{}-{} \"", type, direction);

    ofstream file;

    file.open(m_filename, std::ios_base::app | std::ios_base::in);
    if(!file.is_open()) {
      fmt::print("FAILED: FILE NOT CREATED");
      exit(1);
    }

    file << "{ \n";
    file << test << ": [ \n";

    if ( m_data.size() % 4) {
      exit(1);
    }

    for (int i = 0; i < m_data.size() - 4; i = i + 4) {
      file << "[";
      file << "\n";
      file << m_data.at(i); 
      file << ", \n";
      file << m_data.at(i + 1); 
      file << ", \n";
      file << m_data.at(i + 2); 
      file << ", \n";
      file << m_data.at(i + 3); 
      file << "\n";
      if (i + 3 == m_data.size() - 1) {
        file << "] \n";
      }
      else {
        file << "], \n";
      }
    }

    file << "]";

    file.close();

    m_data.clear();
  }
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif