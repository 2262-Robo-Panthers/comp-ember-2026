// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/MaxSwerveModule.h"

#include "util/Constants.h"
#include "util/Types.h"

using namespace Types::Swerve;
using namespace rev::spark;

MaxSwerveModule::MaxSwerveModule(const MotorPair<Types::CanId> can, const units::radian_t angle) :
  m_motorControllers{
    .drive = SparkMax{can.drive, SparkMax::MotorType::kBrushless},
    .steer = SparkMax{can.steer, SparkMax::MotorType::kBrushless},
  },
  m_encoders{
    .drive = m_motorControllers.drive.GetEncoder(),
    .steer = m_motorControllers.steer.GetAbsoluteEncoder(),
  },
  m_pidLoops{
    .drive = m_motorControllers.drive.GetClosedLoopController(),
    .steer = m_motorControllers.steer.GetClosedLoopController(),
  },
  m_chassisAngularOffset{angle}
{
  m_motorControllers.drive.Configure(
    driveConfig.Get(),
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );
  m_motorControllers.steer.Configure(
    steerConfig.Get(),
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );

  m_desiredState.angle = units::radian_t{m_encoders.steer.GetPosition()};

  ResetDriveEncoder();
}

frc::SwerveModulePosition MaxSwerveModule::GetPosition() const {
  return {
    .distance = units::meter_t{m_encoders.drive.GetPosition()},
    .angle = units::radian_t{m_encoders.steer.GetPosition()} + m_chassisAngularOffset,
  };
}

frc::SwerveModuleState MaxSwerveModule::GetState() const {
  return {
    .speed = units::meters_per_second_t{m_encoders.drive.GetVelocity()},
    .angle = units::radian_t{m_encoders.steer.GetPosition()} + m_chassisAngularOffset,
  };
}

void MaxSwerveModule::SetState(const frc::SwerveModuleState &desired) {
  frc::SwerveModuleState offsetted{};
  offsetted.speed = desired.speed;
  offsetted.angle = desired.angle - frc::Rotation2d(m_chassisAngularOffset);
  offsetted.Optimize(units::radian_t{m_encoders.steer.GetPosition()});

  m_pidLoops.drive.SetReference(offsetted.speed.value(), SparkMax::ControlType::kVelocity);
  m_pidLoops.steer.SetReference(offsetted.angle.Radians().value(), SparkMax::ControlType::kPosition);

  m_desiredState = desired;
}

void MaxSwerveModule::ResetDriveEncoder() {
  m_encoders.drive.SetPosition(0);
}

Types::LazyMemo<rev::spark::SparkMaxConfig> MaxSwerveModule::driveConfig{[](auto &configs) {
  using namespace Constants::Drive::Swerve;

  constexpr double driveFactor = wheelCircumference.value() / driveMotorReduction;
  constexpr double driveVelocityFf = 60.0 / driveWheelFreeSpeed.value();

  configs
    .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
    .SmartCurrentLimit(50);
  configs.encoder
    .PositionConversionFactor(driveFactor)
    .VelocityConversionFactor(driveFactor / 60.0);
  configs.closedLoop
    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    .Pid(
      Constants::Drive::Swerve::pidLoopDrive.p,
      Constants::Drive::Swerve::pidLoopDrive.i,
      Constants::Drive::Swerve::pidLoopDrive.d
    )
    .VelocityFF(driveVelocityFf)
    .OutputRange(-1, 1);
}};

Types::LazyMemo<rev::spark::SparkMaxConfig> MaxSwerveModule::steerConfig{[](auto &configs) {
  constexpr double steerFactor = 2 * std::numbers::pi;

  configs
    .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
    .SmartCurrentLimit(20);
  configs.absoluteEncoder
    .Inverted(true)
    .PositionConversionFactor(steerFactor)
    .VelocityConversionFactor(steerFactor / 60.0);
  configs.closedLoop
    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
    .Pid(
      Constants::Drive::Swerve::pidLoopSteer.p,
      Constants::Drive::Swerve::pidLoopSteer.i,
      Constants::Drive::Swerve::pidLoopSteer.d
    )
    .OutputRange(-1, 1)
    .PositionWrappingEnabled(true)
    .PositionWrappingInputRange(0, steerFactor);
}};
