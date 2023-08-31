// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;

/** Convenience methods for using the RevRobotics api. */
public class RevUtil {

  /**
   * Accepts a command that returns a REVLibError and if it is not "ok" then print an error to the
   * driver station
   *
   * @param error A REVLibError from any REV API command
   */
  public static void checkRevError(REVLibError error) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(error.toString(), true);
    }
  }

  /**
   * Sets the Status 0,1,2,3 frame rates for a CANSparkMax motor controller
   *
   * @param motor A CANSparkMax motor controller
   * @param desc Description of the motor ("Climber1", "Left Drive", etc)
   */
  public static void setPeriodicFramePeriodLow(CANSparkMax motor, String desc) {
    checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20));
    checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20));
    checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100));
    checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 100));
  }

  /**
   * Sets the Status 0,1,2,3 frame rates for a CANSparkMax motor controller
   *
   * @param motor A CANSparkMax motor controller
   * @param desc Description of the motor ("Climber1", "Left Drive", etc)
   */
  public static void setPeriodicFramePeriodHigh(CANSparkMax motor, String desc) {
    checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10));
    checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20));
    checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20));
    checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 50));
  }
}
