// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>
#include <rev/SparkBase.h>
#include <rev/config/SparkMaxConfig.h>

#include "Types.h"
#include "Constants.h"

namespace Subsystems {

class PidSubsystem : public frc2::SubsystemBase {
public:
  virtual void Periodic() override;

  virtual double GetPosition() const;
  virtual void ResetPosition(double position = 0.0);
  virtual void MovePosition(double change, double min = 0.0, double max = 1.0);
  virtual void GoToPosition(double position, double min = 0.0, double max = 1.0);
  virtual bool IsInPosition(double position, double tolerance = Constants::Movement::positionalEpsilon) const;
  virtual frc2::CommandPtr GoToPosition_Command(double position);
  virtual frc2::CommandPtr GoToPositionBlind_Command(double position);

  virtual void ToggleKillSwitch();
  virtual void ToggleKillSwitch(bool value);

protected:
  PidSubsystem(Types::CanId id, std::function<void(rev::spark::SparkMaxConfig &)> initializeConfig);

  rev::spark::SparkMax m_motorController;
  rev::spark::SparkRelativeEncoder m_encoder;
  rev::spark::SparkClosedLoopController m_pidLoop;

  double m_desiredPosition = 0.0;
  bool m_isNeutralized = false;

  Types::LazyMemo<rev::spark::SparkMaxConfig> m_motorConfig;
};

}
