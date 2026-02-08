// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

/**
 * === CONVENTIONS ===
 *
 * coordinates: north-west-up
 * angles: bank-pitch-yaw (CCW increasing)
 */

namespace Types {

/**
 * Type alias for CAN IDs to avoid exposing implementation
 */
using CanId = int;

/**
 * Different frames of reference for coordinates.
 */
enum class CoordinateSystem {
  ROBOT,
  FIELD,
};

enum PovDirection {
  UP = 0,
  RIGHT = 90,
  DOWN = 180,
  LEFT = 270,
};

/**
 * Data structure for storing PID loop constants.
 */
struct PidCoefficients {
  double p, i, d;
};

/**
 * Encapsulation aliases for swerve drive modules.
 */
namespace Swerve {

  /**
   * Container for the four swerve modules in the drivetrain.
   * Values are ordered according to SetIndices.
   */
  template<typename Widget>
  using Set = std::array<Widget, 4>;

  /**
   * Aliases mapping the four corners to their indexed equivalents.
   */
  enum SetIndices {
    FL, FR, RL, RR,
  };

  /**
   * The motor controllers that each module needs.
   */
  template<typename Widget, typename U = Widget>
  struct MotorPair {
    Widget drive;
    U steer;
  };

}

/**
 * A lazy-evaluated memoizer for runtime configuration structures.
 */
template<typename Widget, typename... U>
class LazyMemo {
public:
  inline LazyMemo(std::function<void(Widget &)> initializer, U... params) :
    m_initializer{initializer},
    m_value{params...}
  {}

  inline Widget &Get() {
    if (!m_isInitialized) {
      m_initializer(m_value);
      m_isInitialized = true;
    }

    return m_value;
  }

  inline Widget &Reset() {
    m_initializer(m_value);
    m_isInitialized = true;

    return m_value;
  }

private:
  std::function<void(Widget &)> m_initializer;
  bool m_isInitialized = false;

  Widget m_value;
};

}
