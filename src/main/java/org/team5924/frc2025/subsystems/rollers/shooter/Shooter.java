/*
 * Shooter.java
 */

/* 
 * Copyright (C) 2024-2025 Team 5924 - Golden Gate Robotics and/or its affiliates.
 *
 * This file, and the associated project, are offered under the GNU General
 * Public License v3.0. A copy of this license can be found in LICENSE.md
 * at the root of this project.
 *
 * If this file has been seperated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025.subsystems.rollers.shooter;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystem;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystem.VoltageGoal;
import org.team5924.frc2025.util.LoggedTunableNumber;

@Getter
@Setter
public class Shooter extends GenericRollerSystem<Shooter.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageGoal {
    EMPTY(() -> 0.0),
    LOADING(new LoggedTunableNumber("CoralShooter/LoadingVoltage", 12.0)),
    SHOOTING(new LoggedTunableNumber("CoralShooter/ShootinggVoltage", -12.0)),
    HOLDING(new LoggedTunableNumber("CoralShooter/HoldingVoltage", 0.0));
    private final DoubleSupplier voltageSupplier;
  }

  private Goal goal = Goal.EMPTY;

  public Shooter(ShooterIO io) {
    super("Shooter", io);
  }

  /**
   * Sets the robot's goal.
   *
   * @param newGoal the robot's new goal
   * @throws Exception if the state change fails
   */
  public void setGoal(Goal newGoal) throws Exception {
    // Checking for exceptions to throw
    switch (newGoal) { // example exceptions are shown below
      case EMPTY:
        break;

      case HOLDING:
        break;

      case LOADING:
        if (goal == Goal.HOLDING) throw new Exception("Cannot load while holding!");
        break;

      case SHOOTING:
        if (goal == Goal.EMPTY) throw new Exception("Cannot shoot while shooter is empty!");
        break;

      default:
        throw new Exception("Invalid state: " + newGoal.name());
    }

    // If there are no exceptions thrown, newState will replace currentState
    goal = newGoal;
  }

  /**
   * Gets the robot's goal
   *
   * @return the robot's goal
   */
  @Override
  public Goal getGoal() {
    return goal;
  }
}
