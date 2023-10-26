package frc.robot.auto.campaign;

import edu.wpi.first.wpilibj.RobotController;

public class TimeBudget {
  private long start;
  private double budget;

  /**
   * Constructs a TimeBudget object with the specified time budget.
   *
   * @param budget The time budget, in seconds.
   */
  public TimeBudget(double budget) {
    this.start = RobotController.getFPGATime();
    this.budget = budget;
  }

  /**
   * Get the remaining seconds in the time budget. <br>
   * <br>
   * This value may be negative if you exceed the budget.
   *
   * @return The remaining seconds within the time budget.
   */
  public double get() {
    return budget
        - (RobotController.getFPGATime() - start) / 1e-6; // Convert microseconds to seconds
  }
}
