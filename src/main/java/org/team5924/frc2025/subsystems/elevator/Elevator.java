/*
 * Elevator.java
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

package org.team5924.frc2025.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Position setpoints in meters
  private static final double INTAKE_HEIGHT = 0.0;
  private static final double L1_HEIGHT = 0.5;
  private static final double L2_HEIGHT = 1.0;
  private static final double L3_HEIGHT = 1.5;
  private static final double L4_HEIGHT = 2.0;

  // Tolerance for position control (in meters)
  private static final double POSITION_TOLERANCE = 0.02;

  /** Creates a new elevator. */
  private final ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public enum ElevatorState {
    INTAKE,
    L1,
    L2,
    L3,
    L4,
    MOVING_TO_SETPOINT
  }

  private ElevatorState goalState;
  private ElevatorState state;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.goalState = ElevatorState.INTAKE;
    this.state = ElevatorState.INTAKE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // State machine logic
    updateState();
    // Update motor output based on current state
    updateMotorOutput();
  }

  private void updateState() {
    if (state != goalState && isAtPosition(getTargetHeight(goalState))) {
      state = goalState;
    } else if (state != goalState) {
      state = ElevatorState.MOVING_TO_SETPOINT;
    }
  }

  private void updateMotorOutput() {
    double targetHeight = getTargetHeight(goalState);
    // Use position control instead of voltage control
    io.setPosition(targetHeight);

    // Log the current state and goals
    Logger.recordOutput("Elevator/CurrentState", state.toString());
    Logger.recordOutput("Elevator/GoalState", goalState.toString());
    Logger.recordOutput("Elevator/TargetHeight", targetHeight);
  }

  private double getTargetHeight(ElevatorState state) {
    double height =
        switch (state) {
          case INTAKE -> INTAKE_HEIGHT;
          case L1 -> L1_HEIGHT;
          case L2 -> L2_HEIGHT;
          case L3 -> L3_HEIGHT;
          case L4 -> L4_HEIGHT;
          case MOVING_TO_SETPOINT -> getTargetHeight(goalState);
        };
    return height;
  }

  private boolean isAtPosition(double targetHeight) {
    return Math.abs(inputs.elevatorPositionMeters - targetHeight) < POSITION_TOLERANCE;
  }

  public boolean atGoalPosition() {
    return isAtPosition(getTargetHeight(goalState));
  }

  public void setGoalState(ElevatorState goalState) {
    this.goalState = goalState;
  }
}
