// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/PidSubsystem.h"

#include <functional>

#include <frc2/command/Commands.h>

#include "util/Constants.h"

using namespace rev::spark;

namespace Subsystems {

// TODO motor config fancy smart MAX motion
PidSubsystem::PidSubsystem(Types::CanId id, std::function<void(SparkMaxConfig &)> initializeConfig) :
  m_motorController{id, SparkMax::MotorType::kBrushless},
  m_encoder{m_motorController.GetEncoder()},
  m_pidLoop{m_motorController.GetClosedLoopController()},
  m_motorConfig{initializeConfig}
{
  m_motorController.Configure(
    m_motorConfig.Get(),
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );

  m_encoder.SetPosition(0.0),
  m_pidLoop.SetReference(0.0, SparkLowLevel::ControlType::kPosition);
}

void PidSubsystem::Periodic() {
  if (!m_isNeutralized) {
    m_pidLoop.SetReference(m_desiredPosition, SparkLowLevel::ControlType::kPosition);
  }
}

double PidSubsystem::GetPosition() const {
  return m_encoder.GetPosition();
}

void PidSubsystem::ResetPosition(double position) {
  m_desiredPosition = position;
  m_encoder.SetPosition(position);
  m_pidLoop.SetIAccum(0.0);
  m_pidLoop.SetReference(position, SparkLowLevel::ControlType::kPosition);
}

void PidSubsystem::MovePosition(double change, double min, double max) {
  GoToPosition(m_desiredPosition + change, min, max);
}

void PidSubsystem::GoToPosition(double position, double min, double max) {
  m_desiredPosition = std::clamp(position, min, max); // TODO add override limits?
}

bool PidSubsystem::IsInPosition(double position, double tolerance) const {
  return tolerance < 0.0 || std::abs(GetPosition() - position) <= tolerance;
}

frc2::CommandPtr PidSubsystem::GoToPosition_Command(double position) {
  return
    RunOnce(
      [this, position]() {
        GoToPosition(position);
      }
    )

    .AndThen(
    frc2::cmd::Race(

      frc2::cmd::WaitUntil(
        [this, position]() {
          return IsInPosition(position);
        }
      ),
      frc2::cmd::Wait(Constants::Movement::commandTimeout)
        .AndThen(frc2::cmd::RunOnce(
          [this]() {
            ResetPosition(m_encoder.GetPosition());
            FRC_ReportWarning("Movement command timed out.");
          }
        ))

    ));
}

frc2::CommandPtr PidSubsystem::GoToPositionBlind_Command(double position) {
  return RunOnce(
    [this, position]() {
      GoToPosition(position);
    }
  );
}

void PidSubsystem::ToggleKillSwitch() {
  ToggleKillSwitch(!m_isNeutralized);
}

void PidSubsystem::ToggleKillSwitch(bool value) {
  if (value) {
    m_isNeutralized = true;
  
    m_motorConfig.Get().closedLoop
      .Pid(0.0, 0.0, 0.0);
  }
  else {
    m_isNeutralized = false;

    m_motorConfig.Reset();

    ResetPosition(m_encoder.GetPosition());
  }

  m_motorController.Configure(
    m_motorConfig.Get(),
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );
}

}
