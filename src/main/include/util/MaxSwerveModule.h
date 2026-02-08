// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>

#include <units/angle.h>
#include <units/velocity.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include "Types.h"

class MaxSwerveModule {
public:
  /**
   * @param canIds               CAN IDs of the module's motor controllers.
   * @param chassisAngularOffset Rotation of the module relative to the front-right module, CW being positive.
   */
  MaxSwerveModule(const Types::Swerve::MotorPair<Types::CanId> canIds, const units::radian_t chassisAngularOffset);

  /**
   * @returns The distance traveled by the drive and the angle of the steer.
   */
  frc::SwerveModulePosition GetPosition() const;

  /**
   * @returns The linear speed of the drive and the angle of the steer.
   */
  frc::SwerveModuleState GetState() const;

  /**
   * Requests the module to use a specified state.
   *
   * @param desired New state to set the module to.
   */
  void SetState(const frc::SwerveModuleState &desired);

  /**
   * Resets the drive encoder position to zero.
   */
  void ResetDriveEncoder();

private:
  Types::Swerve::MotorPair<rev::spark::SparkMax> m_motorControllers;
  Types::Swerve::MotorPair<rev::spark::SparkRelativeEncoder, rev::spark::SparkAbsoluteEncoder> m_encoders;
  Types::Swerve::MotorPair<rev::spark::SparkClosedLoopController> m_pidLoops;

  frc::SwerveModuleState m_desiredState{0.0_mps, 0.0_deg};

  const units::radian_t m_chassisAngularOffset;

  static Types::LazyMemo<rev::spark::SparkMaxConfig> driveConfig;
  static Types::LazyMemo<rev::spark::SparkMaxConfig> steerConfig;
};
