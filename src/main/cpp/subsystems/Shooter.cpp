// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

namespace Subsystems {

Shooter::Shooter() :
  m_ctrlMain{
    Constants::Shooter::Can::main,
    rev::spark::SparkFlex::MotorType::kBrushless
  },
    m_ctrlFollower{
    Constants::Shooter::Can::follower,
    rev::spark::SparkFlex::MotorType::kBrushless
  }
{
  using namespace rev::spark;

  m_configMain
    .SetIdleMode(SparkFlexConfig::IdleMode::kBrake)
    .SmartCurrentLimit(20);
  m_configMain.encoder
    .PositionConversionFactor(67); // TODO get actual value
  m_configMain.closedLoop
    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    .Pid(
      67, 67, 67 // TODO do ts :wilted_rose:
    )
    .OutputRange(-1, 1);

  m_ctrlMain.Configure(
    m_configMain,
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );

  m_configFollower
    .SetIdleMode(SparkFlexConfig::IdleMode::kBrake)
    .SmartCurrentLimit(20)
    .Follow(m_ctrlMain, true);

  m_ctrlFollower.Configure(
    m_configFollower,
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );
}

void Shooter::Periodic() {}

void Shooter::setSpeed(double speed) {
  m_ctrlMain.Set(speed);
}
