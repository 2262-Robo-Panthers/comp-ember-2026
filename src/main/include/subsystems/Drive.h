// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADIS16470_IMU.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc2/command/SubsystemBase.h>

#include <util/Types.h>
#include <util/MaxSwerveModule.h>

namespace Subsystems {

class Drive : public frc2::SubsystemBase {
public:
  Drive();

  void Periodic() override;

  /**
   * @returns Whether we should flip the coordinates (i.e. we're on red alliance).
   */
  static bool ShouldFlipCoordinates();

  /**
   * Drives the robot at given translational (x, y) and rotational (w) speeds in the interval [-1, 1].
   *
   * @param x              Speed of the robot in the x axis.
   * @param y              Speed of the robot in the y axis.
   * @param w              Speed of the robot's rotation around the z axis. (poor man's omega)
   * @param referenceFrame Coordinate system to use.
   */
  void SetMotion(double x, double y, double w, Types::CoordinateSystem referenceFrame);

  /**
   * @returns The chassis speeds of the robot calculated from module states.
   */
  const frc::ChassisSpeeds GetChassisSpeeds() const;

  /**
   * Helper function to drive the robot according to calculated chassis speeds.
   */
  void SetChassisSpeeds(const frc::ChassisSpeeds &speeds);

  /**
   * Puts the wheels into an X formation to prevent the robot from getting pushed.
   */
  void DoXFormation();

  /**
   * Resets the drive encoder positions to zero.
   */
  void ResetDriveEncoders();

  /**
   * @returns The driving/steering positions of each of the swerve drive modules.
   */
  const Types::Swerve::Set<frc::SwerveModulePosition> GetModulePositions() const;

  /**
   * @returns The driving/steering positions of each of the swerve drive modules.
   */
  const Types::Swerve::Set<frc::SwerveModuleState> GetModuleStates() const;

  /**
   * Sets the driving/steering states of each of the swerve drive modules.
   */
  void SetModuleStates(const Types::Swerve::Set<frc::SwerveModuleState> states);

  /**
   * @returns The robot heading from the gyroscope.
   */
  units::degree_t GetGyroHeading() const;

  /**
   * @returns The robot turn rate from the gyroscope.
   */
  units::degrees_per_second_t GetGyroTurnRate() const;

  /**
   * Defines the robot's current heading to be the gyroscope's zero.
   */
  void ResetGyroHeading();

  /**
   * Defines the robot's current state to be some pose passed as an argument.
   *
   * @param pose The pose that we define the robot's current state to be
   */
  void ResetOdometry(const frc::Pose2d pose);

  /**
   * Defines the robot's current translation to be some translation passed as an argument.
   *
   * @param translation The translation that we define the robot's current state to be
   */
  void ResetOdometryTranslation(const frc::Translation2d translation);

  /**
   * Defines the robot's current rotation to be some rotation passed as an argument.
   *
   * @param Rotation The rotation that we define the robot's current state to be
   */
  void ResetOdometryRotation(const frc::Rotation2d rotation);

  /**
   * @returns The robot's idea of where it is.
   */
  frc::Pose2d GetOdometry() const;

  /**
   * Enables or disables x-formation.
   */
  void SetXFormation(bool state);

  /**
   * @returns Whether x-formation is currently active.
   */
  bool GetXFormation() const;

  /**
   * @returns The robot's drive kinematics configuration.
   */
  frc::SwerveDriveKinematics<4> &GetKinematics();

private:
  frc::SwerveDriveKinematics<4> m_driveKinematics;
  Types::Swerve::Set<MaxSwerveModule> m_swerveModules;
  frc::ADIS16470_IMU m_gyroscope;
  frc::SwerveDriveOdometry<4> m_odometry;

  bool m_isXFormation = false;
};

}
