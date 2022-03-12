// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
    WPI_TalonSRX m_talon = {32};

    frc::AnalogPotentiometer m_pot{0, 2*M_PI, -M_PI};

    std::vector<double> m_data;

    std::string m_testType = "quasistatic";
    std::string m_direction = "forward";

    double m_motorVoltage; 

    double m_lastPosition = 0.0;

    double m_stepVoltage = 3.0;

    double m_rampRate = 0.1; 

    double m_startTime = 0;
};