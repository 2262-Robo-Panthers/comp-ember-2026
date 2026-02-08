// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/SparkFlex.h>
#include <rev/config/SparkFlexConfig.h>

#include "util/Constants.h"

namespace Subsystems {

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  void setSpeed(double speed);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::spark::SparkFlex m_ctrlMain;
  rev::spark::SparkFlex m_ctrlFollower;

  rev::spark::SparkFlexConfig m_configMain;
  rev::spark::SparkFlexConfig m_configFollower;
};

}
