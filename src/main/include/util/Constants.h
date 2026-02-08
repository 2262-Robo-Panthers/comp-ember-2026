#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include "Types.h"

namespace Constants {

namespace Operator {

  namespace Usb {
    constexpr int driveTrain = 0;
    constexpr int endEffector = 1;
  }

  constexpr double joystickEpsilon = 0.07;
}

namespace Drive {

  namespace Can {
    using Types::CanId;

    inline constexpr std::array swerve{
      Types::Swerve::MotorPair<CanId>{22, 27},
      Types::Swerve::MotorPair<CanId>{26, 31},
      Types::Swerve::MotorPair<CanId>{29, 30},
      Types::Swerve::MotorPair<CanId>{20, 21},
    };
  }

  constexpr auto maxSpeedLinear = 3.0_mps;
  constexpr auto maxAccelLinear = 1.5_mps_sq;
  constexpr auto maxSpeedAngular = 360_deg_per_s;
  constexpr auto maxAccelAngular = 180_deg_per_s_sq;

  namespace Swerve {
    constexpr Types::PidCoefficients pidLoopDrive{
      .p = 0.04,
      .i = 0.0,
      .d = 0.0,
    };

    constexpr Types::PidCoefficients pidLoopSteer{
      .p = 1.0,
      .i = 0.0,
      .d = 0.0,
    };

    constexpr frc::Translation2d wheelDistance{25.5_in, 25.5_in};
    constexpr units::meter_t wheelDiameter = 3_in;
    constexpr units::meter_t wheelCircumference = wheelDiameter * std::numbers::pi;

    constexpr int spurGearTeeth = 22;
    constexpr int bevelGearTeeth = 45;
    constexpr int bevelPinionTeeth = 15;
    constexpr int drivePinionTeeth = 12;
    constexpr double driveMotorReduction = (double)
      (bevelGearTeeth * spurGearTeeth) / (bevelPinionTeeth * drivePinionTeeth);

    constexpr auto driveMotorFreeSpeed = 5676_rpm;
    constexpr auto driveWheelFreeSpeed =
      driveMotorFreeSpeed * wheelCircumference.value() / driveMotorReduction;

    // Translation is relative to drivetrain center, rotation is relative to front right corner.
    inline constexpr std::array moduleOffsets{
      frc::Pose2d{wheelDistance.X() /  2, wheelDistance.Y() /  2,  90_deg},
      frc::Pose2d{wheelDistance.X() /  2, wheelDistance.Y() / -2,   0_deg},
      frc::Pose2d{wheelDistance.X() / -2, wheelDistance.Y() /  2, 180_deg},
      frc::Pose2d{wheelDistance.X() / -2, wheelDistance.Y() / -2, -90_deg},
    };

    inline constexpr std::array xFormation{
      frc::SwerveModuleState{0_mps,  45_deg},
      frc::SwerveModuleState{0_mps, -45_deg},
      frc::SwerveModuleState{0_mps, -45_deg},
      frc::SwerveModuleState{0_mps,  45_deg},
    };
  }

  constexpr auto bumperThickness = 3_in;
  constexpr frc::Translation2d robotSizeInner{29_in, 29_in};
  constexpr frc::Translation2d robotSizeOuter{
    robotSizeInner + frc::Translation2d{bumperThickness, bumperThickness}
  };
}


namespace Movement {
  constexpr double positionalEpsilon = 0.03;
  constexpr auto commandTimeout = 2.0_s;
}

}
