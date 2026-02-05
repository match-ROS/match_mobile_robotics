#pragma once

/**
 * @file fuzzy_types.hpp
 * @brief Type definitions for the Fuzzy Gain Scheduler component.
 *
 * This file contains configuration structures and output types used by
 * the FuzzyGainScheduler for dynamic PI gain scheduling.
 */

#include <string>

namespace cartesian_velocity_controller
{

/**
 * @struct FuzzyGainSchedulerConfig
 * @brief Configuration for the fuzzy gain scheduler.
 *
 * The fuzzy gain scheduler dynamically adjusts PI gains based on tracking error:
 * - Large error: High P gain, zero I gain (fast approach)
 * - Medium error: Medium P gain, low I gain (transition)
 * - Small error: Low P gain, high I gain (precise convergence)
 * - Very small error: Zero gains (dead-band)
 */
struct FuzzyGainSchedulerConfig
{
  /// Enable/disable the fuzzy gain scheduler
  bool enabled{true};

  // ============== Output Gain Ranges for Position ==============

  /// Minimum proportional gain for position [0, position_p_gain_max]
  double position_p_gain_min{0.3};
  /// Maximum proportional gain for position
  double position_p_gain_max{3.0};
  /// Minimum integral gain for position
  double position_i_gain_min{0.0};
  /// Maximum integral gain for position
  double position_i_gain_max{2.0};

  // ============== Output Gain Ranges for Orientation ==============

  /// Minimum proportional gain for orientation
  double orientation_p_gain_min{0.2};
  /// Maximum proportional gain for orientation
  double orientation_p_gain_max{1.5};
  /// Minimum integral gain for orientation
  double orientation_i_gain_min{0.0};
  /// Maximum integral gain for orientation
  double orientation_i_gain_max{1.0};

  // ============== Dead-band with Hysteresis ==============

  /// Enable dead-band functionality
  bool dead_band_enabled{true};
  /// Position error threshold to ENTER dead-band [m]
  double position_dead_band_enter{0.001};
  /// Position error threshold to EXIT dead-band [m] (must be > enter for hysteresis)
  double position_dead_band_exit{0.002};
  /// Orientation error threshold to ENTER dead-band [rad]
  double orientation_dead_band_enter{0.01};
  /// Orientation error threshold to EXIT dead-band [rad] (must be > enter)
  double orientation_dead_band_exit{0.02};

  // ============== Smooth Transition ==============

  /// Time constant for velocity_scale low-pass filter [s]
  double velocity_scale_smoothing_time{0.1};

  // ============== Rate Limiting for Stability ==============

  /// Enable rate limiting on gain changes
  bool gain_rate_limiting_enabled{true};
  /// Maximum gain change rate [gain_units/second]
  double max_gain_change_rate{2.0};

  // ============== Integral Management during Ki Transitions ==============

  /// If Ki increases by more than this threshold, reset integral accumulator
  double ki_reset_threshold{0.5};
  /// Scale integral proportionally when Ki changes (to maintain I contribution)
  bool scale_integral_on_ki_change{true};

  // ============== FLL Rules File (MANDATORY) ==============

  /// Path to the .fll file containing fuzzy rules (required)
  /// Example: "$(find cartesian_velocity_controller)/config/fuzzy_gain_scheduler.fll"
  std::string rules_file{""};
};

/**
 * @struct FuzzyGainOutput
 * @brief Output from the fuzzy gain scheduler.
 *
 * This structure contains all computed values that the AttractiveController
 * needs to apply the fuzzy-scheduled gains.
 */
struct FuzzyGainOutput
{
  // ============== Computed Gains (rate-limited) ==============

  /// Proportional gain for position
  double position_p_gain{1.0};
  /// Integral gain for position
  double position_i_gain{0.0};
  /// Proportional gain for orientation
  double orientation_p_gain{0.5};
  /// Integral gain for orientation
  double orientation_i_gain{0.0};

  // ============== Dead-band (smooth transition) ==============

  /// Velocity scale factor [0, 1]: 1.0 = full speed, 0.0 = stopped (dead-band)
  double velocity_scale{1.0};
  /// True when both position and orientation errors are below dead-band thresholds (with hysteresis)
  bool in_dead_band{false};

  // ============== Integral Management (to be applied by AttractiveController) ==============

  /// If true, reset position integral accumulator
  bool should_reset_position_integral{false};
  /// If true, reset orientation integral accumulator
  bool should_reset_orientation_integral{false};
  /// Scale factor for position integral [0, 1] (used when Ki changes)
  double position_integral_scale{1.0};
  /// Scale factor for orientation integral [0, 1]
  double orientation_integral_scale{1.0};
};

}  // namespace cartesian_velocity_controller

