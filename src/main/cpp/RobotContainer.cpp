// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

#include "util/Constants.h"

#define AXIS(ctrl, bind) frc::ApplyDeadband(ctrl.bind(), joystickEpsilon)
#define Y_AXIS(ctrl, bind) (-AXIS(ctrl, bind))

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  using Constants::Operator::joystickEpsilon;

  /***
   *** Drive
   ***/

  m_subsystemDrive.SetDefaultCommand(
    frc2::cmd::Run(
      [this]() {
        m_subsystemDrive.SetMotion(
          Y_AXIS(m_driveTrainController, GetRightY),
          -AXIS(m_driveTrainController, GetRightX), // negative: robot X-axis points left
          -AXIS(m_driveTrainController, GetLeftX), // negative: positive angles go left
          Types::CoordinateSystem::FIELD
        );
      },
      {&m_subsystemDrive}
    )
  );

  m_driveTrainController.Back().OnTrue(
    frc2::cmd::RunOnce(
      [this]() {
        m_subsystemDrive.SetXFormation(false);
      },
      {&m_subsystemDrive}
    )
  );

  m_driveTrainController.Start().OnTrue(
    frc2::cmd::RunOnce(
      [this]() {
        m_subsystemDrive.SetXFormation(true);
      },
      {&m_subsystemDrive}
    )
  );

  m_driveTrainController.A().OnTrue(
    frc2::cmd::RunOnce(
      [this]() {
        m_subsystemDrive.ResetOdometryRotation(0_deg);
      },
      {&m_subsystemDrive}
    )
  );

  m_driveTrainController.B().OnTrue(
    frc2::cmd::RunOnce(
      [this]() {
        m_subsystemDrive.ResetOdometryRotation(180_deg);
      },
      {&m_subsystemDrive}
    )
  );

  m_driveTrainController.X().OnTrue(
    frc2::cmd::RunOnce(
      [this]() {
        m_subsystemDrive.ResetOdometryTranslation({0_m, 0_m});
      },
      {&m_subsystemDrive}
    )
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
