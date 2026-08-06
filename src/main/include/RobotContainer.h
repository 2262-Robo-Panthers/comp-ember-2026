// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/CommandXboxController.h>

#include "subsystems/Drive.h"
#include "subsystems/Shooter.h"

#include "util/Constants.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command *GetAutonomousCommand();

 private:
  void ConfigureBindings();
  void ConfigureNamedCommands();

  frc2::CommandXboxController m_driveTrainController{Constants::Operator::Usb::driveTrain};
  frc2::CommandXboxController m_endEffectorController{Constants::Operator::Usb::endEffector};

  Subsystems::Drive m_subsystemDrive;
  Subsystems::Shooter m_subsystemShooter;

  frc::SendableChooser<frc2::Command *> m_autoChooser;
};
