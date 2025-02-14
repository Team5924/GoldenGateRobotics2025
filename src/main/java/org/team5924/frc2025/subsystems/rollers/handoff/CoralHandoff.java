/*
 * CoralHandoff.java
 */

/* 
 * Copyright (C) 2024-2025 Team 5924 - Golden Gate Robotics and/or its affiliates.
 *
 * This file, and the associated project, are offered under the GNU General
 * Public License v3.0. A copy of this license can be found in LICENSE.md
 * at the root of this project.
 *
 * If this file has been separated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025.subsystems.rollers.handoff;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystem;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystem.VoltageState;
import org.team5924.frc2025.util.LoggedTunableNumber;

@Getter
@Setter
public class CoralHandoff extends GenericRollerSystem<CoralHandoff.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageState {
    OUTPUT(new LoggedTunableNumber("CoralHandoff/OutputVoltage", 12.0)),
    LOADING(new LoggedTunableNumber("CoralHandoff/LoadingVoltage", 12.0)),
    HOLDING(new LoggedTunableNumber("CoralHandoff/HoldingVoltage", 0.0)),
    EMPTY(new LoggedTunableNumber("CoralHandoff/EmptyVoltage", 0.0)),
    REVERSE(new LoggedTunableNumber("CoralHandoff/ReverseVoltage", -12.0));
    private final DoubleSupplier voltageSupplier;
  }

  private Goal goal = Goal.EMPTY;

  public CoralHandoff(CoralHandoffIO io) {
    super("CoralHandoff", io);
  }

  /**
   * Get if the robot has coral.
   *
   * @return true if the robot has coral
   */
  public boolean hasCoral() {
    return true;
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
      case OUTPUT:
        if (goal == Goal.EMPTY) throw new Exception("Cannot output while handoff is empty!");
        break;

      case LOADING:
        if (goal == Goal.HOLDING) throw new Exception("Cannot load while holding!");
        break;

      case HOLDING:
        break;

      case EMPTY:
        break;

      case REVERSE:
        break;

      default:
        throw new Exception("Invalid handoff state: " + newGoal.name());
    }
    // TODO: replace example exceptions with real exceptions

    // If there are no exceptions thrown, newState will replace currentState
    goal = newGoal;
  }

  /**
   * Gets the robot's goal
   *
   * @return the robot's goal
   */
  @Override
  public Goal getGoalState() {
    return goal;
  }
}
