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
  * If this file has been seperated from the original project, you should have
  * received a copy of the GNU General Public License along with it.
  * If you did not, see <https://www.gnu.org/licenses>.
  */

package org.team5924.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Tolerance for position control (in meters)
  private static final double POSITION_TOLERANCE = 0.02;
  private static final Distance SPROCKET_RADIUS = Inches.of(2);
  private static final double RATIO = 3;

  /** Creates a new elevator. */
  private final ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public enum ElevatorState {
    INTAKE(0.0),
    L1(0.5),
    L2(1.0),
    L3(1.5),
    L4(2),
    MOVING(-1);

    private final double height;

    ElevatorState(double height) {
      this.height = height;
    }
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
    runStateMachine();
  }

  private void runStateMachine() {
    Logger.recordOutput("Elevator/CurrentState", state.toString());
    Logger.recordOutput("Elevator/GoalState", goalState.toString());
    Logger.recordOutput("Elevator/TargetHeight", goalState.height);

    if (state == ElevatorState.MOVING && isAtSetpoint()) {
      state = goalState;
    }
  }

  private double getElevatorPositionMeters() {
    return Radians.of(inputs.leftPositionRads).in(Rotations)
        * 2
        * Math.PI
        * SPROCKET_RADIUS.in(Meters)
        / RATIO;
  }

  private boolean isAtSetpoint() {
    return Math.abs(getElevatorPositionMeters() - this.goalState.height) < POSITION_TOLERANCE;
  }

  public void setGoalState(ElevatorState goalState) {
    this.goalState = goalState;
    this.state = ElevatorState.MOVING;
    if (goalState != ElevatorState.MOVING) {
      io.setPosition(goalState.height);
    } else {
      DriverStation.reportError("Invalid goal ElevatorState!", null);
    }
  }
}
