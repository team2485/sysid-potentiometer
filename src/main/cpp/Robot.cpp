// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "ctre/Phoenix.h"

#include "math.h"

#include <fmt/core.h>

#include <frc/AnalogPotentiometer.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "SysIdLogger.h"

#include "SysIdGeneralMechanismLogger.h"

using namespace frc;

WPI_TalonSRX m_talonsrx = {32};

frc::AnalogPotentiometer analogPot{0, 2*M_PI, -M_PI};

std::vector<double> m_data;

double m_motorVoltage = 3.0; 




Robot::Robot() : frc::TimedRobot(0.005){
  try {
    m_json = sysid::GetConfigJson();

    sysid::AddMotorController(m_talonsrx, "talonSRX", false, &m_talonsrx);
  } catch (std::exception& e) {
    fmt::print("FAILED: \n", e.what());
    std::exit(-1);
  }
}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */

void SysIdLogger::SendData() {
  std::stringstream ss;
  for (size_t i = 0; i < m_data.size(); ++i) {
    ss << std::to_string(m_data[i]);
    if (i < m_data.size() - 1) {
      ss << ",";
    }
  }

  std::string m_testType == "Dynamic" ? "fast" : "slow";
  std::string direction = m_motorVoltage > 0 ? "forward" : "backward";
  std::string test = fmt::format("{}-{}", type, direction);

}
void Robot::AutonomousInit() {

  //NT_Logger.InitLogging()
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
    m_data.clear();

  } else {
    // Default Auto goes here
  }
  double m_starttime = frc::Timer::GetFPGATimestamp().value();
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
    std::string direction = m_motorVoltage > 0 ? "forward" : "backward";
    if (direction == "forward") {
      m_motorVoltage = m_motorVoltage;
    }
    else if (direction == "backward") {
      m_motorVoltage = -m_motorVoltage;
    }
  } else {
    // Default Auto goes here
  }
}

// void SysIdGeneralMechanismLogger::Log(double voltage, double measuredPosition, double measuredVelocity) {
//     UpdateData();
//     if (m_data.size() < 3600) {
//       std::array<double, 4> arr = {__time_t, voltage, measuredPosition, 
//       measuredVelocity};

//       m_data.insert(m_data.end(), arr.cbegin(), arr.cend());
//     }

//     //m_primaryMotorVoltage = units::volt_t{m_motorVoltage};
// }

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  m_motorVoltage = 0;

    frc::SmartDashboard::PutBoolean("SysIdOverflow", m_data.size() >= 3600);

      std::stringstream ss;
      for (size_t i = 0; i < m_data.size(); ++i) {
        ss << std::to_string(m_data[i]);
        if (i < m_data.size() - 1) {
          ss << ",";
    }
  }
  //implement sd over?
  //NT_Logger.SendData();
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
