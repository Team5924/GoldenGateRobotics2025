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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
  // Tolerance for position control (in meters)
  private static final LoggedTunableNumber POSITION_TOLERANCE =
      new LoggedTunableNumber("Elevator/PositionTolerance", 0.02);

  /** Creates a new elevator. */
  private final ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public enum ElevatorState {
    INTAKE(new LoggedTunableNumber("Elevator/IntakeHeight", 0)),
    L1(new LoggedTunableNumber("Elevator/L1Height", 1)),
    L2(new LoggedTunableNumber("Elevator/L2Height", 0.5)),
    L3(new LoggedTunableNumber("Elevator/L3Height", 1)),
    L4(new LoggedTunableNumber("Elevator/L4Height", 1.5)),
    MOVING(new LoggedTunableNumber("Elevator/MovingHeight", 0)),
    MANUAL(new LoggedTunableNumber("Elevator/ManualHeight", 0));

    private final LoggedTunableNumber heightMeters;

    ElevatorState(LoggedTunableNumber heightMeters) {
      this.heightMeters = heightMeters;
    }
  }

  @Getter private ElevatorState goalState;

  private final Alert leftMotorDisconnected;
  private final Alert rightMotorDisconnected;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.goalState = ElevatorState.MANUAL;
    RobotState.getInstance().setElevatorState(this.goalState);
    this.leftMotorDisconnected =
        new Alert("Left elevator motor disconnected!", Alert.AlertType.kWarning);
    this.rightMotorDisconnected =
        new Alert("Right elevator motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/GoalState", goalState.toString());
    Logger.recordOutput("Elevator/TargetHeight", goalState.heightMeters);

    leftMotorDisconnected.set(!inputs.leftMotorConnected);
    rightMotorDisconnected.set(!inputs.rightMotorConnected);

    io.setHeight(goalState.heightMeters.getAsDouble());
  }

  private double getElevatorPositionMeters() {
    return Radians.of(inputs.leftPositionRads).in(Rotations)
        * 2
        * Math.PI
        * Constants.SPROCKET_RADIUS.in(Meters)
        / Constants.MOTOR_TO_ELEVATOR_REDUCTION;
  }

  // public double rotationsToMeters(double rotations) {
  //   return rotations
  //       * 2
  //       * Math.PI
  //       * SPROCKET_RADIUS.in(Meters)
  //       / Constants.MOTOR_TO_ELEVATOR_REDUCTION;
  // }

  // public static double metersToRotations(double height) {
  //   return height
  //       * Constants.MOTOR_TO_ELEVATOR_REDUCTION
  //       / (2 * Math.PI * SPROCKET_RADIUS.in(Meters));
  // }

  public boolean isAtSetpoint() {
    return Math.abs(getElevatorPositionMeters() - this.goalState.heightMeters.getAsDouble())
        < POSITION_TOLERANCE.getAsDouble();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setSoftStopOn() {
    io.setSoftStopOn();
  }

  public void setSoftStopOff() {
    io.setSoftStopOff();
  }

  public void setGoalState(ElevatorState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case MANUAL -> RobotState.getInstance().setElevatorState(ElevatorState.MANUAL);
      case MOVING -> DriverStation.reportError("Invalid goal ElevatorState!", null);
      default -> {
        RobotState.getInstance().setElevatorState(ElevatorState.MOVING);
        io.setPosition(goalState.heightMeters.getAsDouble());
      }
    }
  }
}
