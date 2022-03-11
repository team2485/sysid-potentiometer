// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

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

using namespace frc;

WPI_TalonSRX m_talonsrx = {32};

frc::AnalogPotentiometer analogPot{0, 2*M_PI, -M_PI};

std::vector<double> m_data;

std::string m_testType; 

double m_motorVoltage = 0.0; 

double m_lastPosition = 0.0;

double stepVoltage = 3.0;

double rampRate = 0.2; 

// tmpfile;

 //std::string sysidLogs = "C:/home/lvuser/sysidLogs/sysid.txt";


Robot::Robot() : frc::TimedRobot(5_ms){
  try {
    //m_json = GetConfigJson();

  } catch (std::exception& e) {
    fmt::print("FAILED: \n", e.what());
    std::exit(-1);
  }
}

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_data.clear();
  double m_starttime = frc::Timer::GetFPGATimestamp().value();
  m_data.push_back(m_starttime);
}

void Robot::AutonomousPeriodic() {
  m_lastPosition = analogPot.Get();
  std::string type = m_testType == "Dynamic" ? "fast" : "slow";
  m_motorVoltage = m_talonsrx.GetMotorOutputVoltage();
  std::string direction = m_motorVoltage > 0 ? "forward" : "backward";
  if (m_testType == "fast") {
    if (direction == "forward") {
      m_motorVoltage = stepVoltage;
      m_talonsrx.setVoltage(m_motorVoltage);
    }
    else {
      m_motorVoltage = -stepVoltage;
      m_talonsrx.setVoltage(m_motorVoltage);
    }

  }
  else {
    if(direction == "forward") {
      m_motorVoltage = rampRate * (frc::Timer::GetFPGATTimestamp().value() - m_data::front());
      m_talonsrx.setVoltage(m_motorVoltage);
    }
    else {
      m_motorVoltage = - rampRate * (frc::Timer::GetFPGATTimestamp().value() - m_data::front());
      m_talonsrx.setVoltage(m_motorVoltage);
    }
  }

  double m_position = analogPot.Get();
  double m_timeStepInit = frc::Timer::Get();
  double m_potVelocity = (m_position - m_lastPosition) / 0.005; 
  
  m_data.insert( m_data.begin(), {type, direction});
  m_data.insert( m_data.end(), {m_talonsrx.GetMotorOutputVoltage(), m_position, m_potVelocity});

  m_lastPosition = analogPot.Get();

}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  m_motorVoltage = 0.0; 
  //frc::SmartDashboard::PutBoolean("SysIdOverflow", m_data.size() >= 3600);
  //string to json
      std::stringstream ss;
      for (int i = 0; i < m_data.size(); ++i) {
        ss << std::to_string(m_data[i]);
        if (i < m_data.size() - 1) {
          ss << ",";
    } 
  }
  //could use test below to prefix test name OR leave it as first term in vector (current V does both)
    std::string type = m_testType == "Dynamic" ? "fast" : "slow";
    std::string direction = m_motorVoltage > 0 ? "forward" : "backward";
    std::string test = fmt::format("{}-{}", type, direction);
    //FILE WRITING ATTEMPT LOLZ
    // fs::ofstream file;
    // fs::file.open("C:/home/lvuser/sysidLogs/sysid.txt");
    // //o << 

    // if (!file.is_open()) {
    //   fmt::print("FAILED");
    // }
    // else {
    //   file.write(m_data.data(), m_data.size());
    //   fmt::print("SUCCEEDED");
    // }

    // file.close();
    const fs::path& path = "/home/lvuser/sysidLogs/sysid.txt";
    fs::create_directories(path().root_directory());

    std::error_code ec;
    wpi::raw_fd_ostream ostream{path.string(), ec};

    if (ec) {
      throw std::runtime_error("FAILED: " + ec.message());
    }

    ostream << ss;


  m_data.clear();

}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
