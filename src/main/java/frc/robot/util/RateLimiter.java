// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

/** Rate limiter to smooth out inclination or declination of a value over time. */
public class RateLimiter {

  private final double inclinationRateLimit;
  private final double declinationRateLimit;
  private double prevVal;
  private double prevTime;

  /**
   * Creates a new RateLimiter with the given rate limit and initial value.
   *
   * @param inclinationRateLimit The rate-of-change positive limit, in units per second.
   * @param declinationRateLimit The rate-of-change negative limit, in units per second.
   * @param initialValue The initial value of the input.
   */
  public RateLimiter(
      double inclinationRateLimit, double declinationRateLimit, double initialValue) {
    this.inclinationRateLimit = inclinationRateLimit;
    this.declinationRateLimit = declinationRateLimit;
    prevVal = initialValue;
    prevTime = WPIUtilJNI.now() * 1e-6;
  }

  /**
   * Creates a new RateLimiter with the given rate limit and an initial value of zero.
   *
   * @param inclinationRateLimit The rate-of-change positive limit, in units per second.
   * @param declinationRateLimit The rate-of-change negativelimit, in units per second.
   */
  public RateLimiter(double inclinationRateLimit, double declinationRateLimit) {
    this(inclinationRateLimit, declinationRateLimit, 0);
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - prevTime;
    prevVal +=
        MathUtil.clamp(
            input - prevVal,
            -declinationRateLimit * elapsedTime,
            inclinationRateLimit * elapsedTime);
    prevTime = currentTime;
    return prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    prevVal = value;
    prevTime = WPIUtilJNI.now() * 1e-6;
  }
}
