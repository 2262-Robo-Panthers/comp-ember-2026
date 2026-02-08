// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"

#include <algorithm>

#include <hal/FRCUsageReporting.h>

#include <frc/ADIS16470_IMU.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/DriverStation.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <units/angle.h>

#include "util/Constants.h"

using frc::ADIS16470_IMU::IMUAxis::kZ;
using Idx = Types::Swerve::SetIndices;
using namespace pathplanner;

namespace Subsystems {

bool Drive::ShouldFlipCoordinates() {
  auto alliance = frc::DriverStation::GetAlliance();
  return alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
}

Drive::Drive() :
  m_driveKinematics{
    Constants::Drive::Swerve::moduleOffsets[Idx::FL].Translation(),
    Constants::Drive::Swerve::moduleOffsets[Idx::FR].Translation(),
    Constants::Drive::Swerve::moduleOffsets[Idx::RL].Translation(),
    Constants::Drive::Swerve::moduleOffsets[Idx::RR].Translation(),
  },
  m_swerveModules{
    MaxSwerveModule{
      Constants::Drive::Can::swerve[Idx::FL],
      Constants::Drive::Swerve::moduleOffsets[Idx::FL].Rotation().Radians(),
    },
    MaxSwerveModule{
      Constants::Drive::Can::swerve[Idx::FR],
      Constants::Drive::Swerve::moduleOffsets[Idx::FR].Rotation().Radians(),
    },
    MaxSwerveModule{
      Constants::Drive::Can::swerve[Idx::RL],
      Constants::Drive::Swerve::moduleOffsets[Idx::RL].Rotation().Radians(),
    },
    MaxSwerveModule{
      Constants::Drive::Can::swerve[Idx::RR],
      Constants::Drive::Swerve::moduleOffsets[Idx::RR].Rotation().Radians(),
    },
  },
  m_odometry{
    m_driveKinematics,
    units::radian_t{m_gyroscope.GetAngle(kZ)},
    GetModulePositions(),
    frc::Pose2d{}
  }
{
  HAL_Report(
    HALUsageReporting::kResourceType_RobotDrive,
    HALUsageReporting::kRobotDriveSwerve_MaxSwerve
  );

  AutoBuilder::configure(
    [this]()            { return GetOdometry();      },
    [this](auto pose)   { ResetOdometry(pose);       },
    [this]()            { return GetChassisSpeeds(); },
    [this](auto speeds) { SetChassisSpeeds(speeds);  },
    std::make_shared<PPHolonomicDriveController>(
      PIDConstants(5.0, 0.0, 0.0),
      PIDConstants(5.0, 0.0, 0.0)
    ),
    RobotConfig::fromGUISettings(),
    Drive::ShouldFlipCoordinates,
    this
  );
}

void Drive::Periodic() {
  m_odometry.Update(GetGyroHeading(), GetModulePositions());
}

void Drive::SetMotion(double x, double y, double w, Types::CoordinateSystem referenceFrame) {
  if (GetXFormation()) {
    DoXFormation();
    return;
  }

  auto xScaled = x * Constants::Drive::maxSpeedLinear;
  auto yScaled = y * Constants::Drive::maxSpeedLinear;
  auto wScaled = w * Constants::Drive::maxSpeedAngular;

  if (ShouldFlipCoordinates()) {
    xScaled = -xScaled;
    yScaled = -yScaled;
  }

  SetChassisSpeeds(
    referenceFrame == Types::CoordinateSystem::FIELD
      ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xScaled, yScaled, wScaled, GetGyroHeading())
      : frc::ChassisSpeeds{xScaled, yScaled, wScaled}
  );
}

const frc::ChassisSpeeds Drive::GetChassisSpeeds() const {
  return m_driveKinematics.ToChassisSpeeds(GetModuleStates());
}

void Drive::SetChassisSpeeds(const frc::ChassisSpeeds &speeds) {
  auto states = m_driveKinematics.ToSwerveModuleStates(speeds);

  // Reverify that the wheel speeds weren't pushed above the maximum after calculations.
  m_driveKinematics.DesaturateWheelSpeeds(&states, Constants::Drive::maxSpeedLinear);

  SetModuleStates(states);
}

void Drive::DoXFormation() {
  SetModuleStates(Constants::Drive::Swerve::xFormation);
}

void Drive::ResetDriveEncoders() {
  for (size_t i = Idx::FL; i <= Idx::RR; ++i) {
    m_swerveModules[i].ResetDriveEncoder();
  }
}

const Types::Swerve::Set<frc::SwerveModulePosition> Drive::GetModulePositions() const {
  return std::move(Types::Swerve::Set<frc::SwerveModulePosition>{
    m_swerveModules[Idx::FL].GetPosition(),
    m_swerveModules[Idx::FR].GetPosition(),
    m_swerveModules[Idx::RL].GetPosition(),
    m_swerveModules[Idx::RR].GetPosition(),
  });
}

const Types::Swerve::Set<frc::SwerveModuleState> Drive::GetModuleStates() const {
  return std::move(Types::Swerve::Set<frc::SwerveModuleState>{
    m_swerveModules[Idx::FL].GetState(),
    m_swerveModules[Idx::FR].GetState(),
    m_swerveModules[Idx::RL].GetState(),
    m_swerveModules[Idx::RR].GetState(),
  });
}

void Drive::SetModuleStates(const Types::Swerve::Set<frc::SwerveModuleState> states) {
  for (size_t i = Idx::FL; i <= Idx::RR; ++i) {
    m_swerveModules[i].SetState(states[i]);
  }
}

units::degree_t Drive::GetGyroHeading() const {
  return m_gyroscope.GetAngle(kZ);
}

units::degrees_per_second_t Drive::GetGyroTurnRate() const {
  return m_gyroscope.GetRate(kZ);
}

void Drive::ResetGyroHeading() {
  m_gyroscope.Reset();
}

void Drive::ResetOdometry(const frc::Pose2d pose) {
  m_gyroscope.SetGyroAngle(kZ, pose.Rotation().Degrees());
  m_odometry.ResetPosition(GetGyroHeading(), GetModulePositions(), pose);
}

void Drive::ResetOdometryTranslation(const frc::Translation2d translation) {
  ResetOdometry(frc::Pose2d{
    translation,
    m_odometry.GetPose().Rotation()
  });
}

void Drive::ResetOdometryRotation(const frc::Rotation2d rotation) {
  ResetOdometry(frc::Pose2d{
    m_odometry.GetPose().Translation(),
    rotation
      + frc::Rotation2d{ShouldFlipCoordinates() ? 180_deg : 0_deg}
  });
}

frc::Pose2d Drive::GetOdometry() const {
  return m_odometry.GetPose();
}

void Drive::SetXFormation(bool state) {
  m_isXFormation = state;
}

bool Drive::GetXFormation() const {
  return m_isXFormation;
}

frc::SwerveDriveKinematics<4> &Drive::GetKinematics() {
  return m_driveKinematics;
}

}
