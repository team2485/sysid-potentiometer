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
  m_lastPosition = analogPot.Get();
  
  if (m_testType == "Dynamic") {
    if (m_direction == "Forward") {
      m_talonsrx.SetVoltage((units::volt_t) stepVoltage);
    }
    else {
      m_talonsrx.SetVoltage((units::volt_t) -stepVoltage);
    }
  }
  else {
    if(m_direction == "Forward") {
      m_talonsrx.SetVoltage((units::volt_t)  rampRate * (frc::Timer::GetFPGATimestamp().value() - m_startTime));
    }
    else {
      m_talonsrx.SetVoltage((units::volt_t) - rampRate * (frc::Timer::GetFPGATimestamp().value() - m_startTime));
    }
  }

  double m_position = analogPot.Get();
  double m_potVelocity = (m_position - m_lastPosition) / 0.005; 
  
    m_data.insert( m_data.end(), {m_talonsrx.GetMotorOutputVoltage(), m_position, m_potVelocity});

  m_lastPosition = analogPot.Get();

}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  m_motorVoltage = 0.0; 
  
    std::string type = m_testType == "Dynamic" ? "fast" : "slow";
    std::string direction = m_motorVoltage > 0 ? "forward" : "backward";
    std::string test = fmt::format("{}-{}", type, direction);

    ofstream MyFile;
    string filename("/home/lvuser/sysidLogs/sysid.json");

    MyFile.open(filename, std::ios_base::app | std::ios_base::in);
      if(!MyFile.is_open()) {
        fmt::print("FAILED: FILE NOT CREATED");
        exit(1);
      }

    MyFile << "{";
    MyFile << test << ": [";

    cout << m_data.size() % 4;

    for (int i = 0; i < m_data.size(); i = i + 4) {
      MyFile << "[";
      MyFile << m_data.at(i); 
      MyFile << m_data.at(i + 1); 
      MyFile << m_data.at(i + 2); 
      MyFile << m_data.at(i + 3); 
      MyFile << "]";
    }

    MyFile << "]";

    MyFile.close();

    m_data.clear();

}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif