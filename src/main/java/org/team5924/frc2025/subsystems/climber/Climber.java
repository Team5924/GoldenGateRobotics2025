/*
 * Climber.java
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

package org.team5924.frc2025.subsystems.climber;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.subsystems.elevator.Elevator.ElevatorState;
import org.team5924.frc2025.subsystems.pivot.AlgaePivot.AlgaePivotState;
import org.team5924.frc2025.util.LoggedTunableNumber;

@Setter
@Getter
public class Climber extends SubsystemBase {
  public enum ClimberState {

    // Default state, will be here most of the match
    STOW(new LoggedTunableNumber("Climber/StowVoltage", 0.0),
      new LoggedTunableNumber("Climber/StowPosition", 0.0)),

    // Lining up climber to cage
    LINEUP(new LoggedTunableNumber("Climber/LineupVoltage", 12.0),
      new LoggedTunableNumber("Climber/LineupPosition", 50 + 90 - 35)),

    // Ready to climb
    READY_TO_CLIMB(new LoggedTunableNumber("Climber/ReadyToClimbVoltage", 0.0),
      new LoggedTunableNumber("Climber/ReadyToClimbPosition", 50 + 90 - 35)),

    // Pulling onto cage, lifting robot
    CLIMB(new LoggedTunableNumber("Climber/ClimbVoltage", 12.0),
      new LoggedTunableNumber("Climber/ClimbPosition", 50 + 90 + 90 + 35));

    private final LoggedTunableNumber volts;
    private LoggedTunableNumber targetPositionDegrees;
    
    ClimberState(LoggedTunableNumber volts, LoggedTunableNumber targetPositionDegrees) {
      this.volts = volts;
      this.targetPositionDegrees = targetPositionDegrees;
    }
  }

  private ClimberState goalState = ClimberState.STOW;
  private ClimberState lastState;

  private final Alert disconnected;

  private final Alert invalidStateTransition;

  protected final Timer stateTimer = new Timer();

  private final ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private static final LoggedTunableNumber laserCanDetectThreshold =
      new LoggedTunableNumber("Climber/LaserCAN/DetectThreshold", 20);

  public Climber(ClimberIO io) {
    this.io = io;

    disconnected = new Alert("Climber motor is disconnected!", Alert.AlertType.kWarning);
    invalidStateTransition = new Alert("Invalid state transition!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    disconnected.set(!inputs.motorConnected);


    if (isCageInClimber()
        && RobotState.getInstance().getAlgaePivotState() == AlgaePivotState.INTAKE_FLOOR
        && RobotState.getInstance().getElevatorPositionMeters()
            <= (ElevatorState.L1).getHeightMeters().getAsDouble() + 0.02) {
      
      // If the robot's state is STOW && the cage is within range && algae pivot is STOW &&
      // elevator height is below L1 elevator height, then set the robot's state to READY_TO_CLIMB
      if (getGoalState() == ClimberState.STOW)
          setGoalState(ClimberState.READY_TO_CLIMB);
    } else if (getGoalState() == ClimberState.READY_TO_CLIMB) {
      // If the requirements for READY_TO_CLIMB are not met and the robot's state is READY_TO_CLIMB,
      // then set the robot's state to STOW
      setGoalState(ClimberState.STOW);
    }

    if (getGoalState() != lastState) {
      stateTimer.reset();
      lastState = getGoalState();
    }


    double currentPositionDegrees = Units.radiansToDegrees(inputs.positionRads);
    double rotationDifference = goalState.targetPositionDegrees.getAsDouble() - currentPositionDegrees;
    if (rotationDifference < Constants.CLIMBER_THRESHOLD_DEG) {
      // run the motor in the direction of the target position
      io.runVolts(Math.signum(rotationDifference) * goalState.volts.getAsDouble());
    }


    Logger.recordOutput("Climber/Climber Goal", goalState.toString());
  }

  /**
   * Sets the goal state of the climber.
   *
   * @param newGoal the new goal state
   */
  public void setGoalState(ClimberState newGoal) {
    this.goalState = newGoal;
    switch (goalState) {
      case STOW:
        RobotState.getInstance().setClimberState(ClimberState.STOW);
        break;

      case LINEUP:
          RobotState.getInstance().setClimberState(ClimberState.LINEUP);
          break;

      case READY_TO_CLIMB:
        RobotState.getInstance().setClimberState(ClimberState.READY_TO_CLIMB);
        break;

      case CLIMB:
        // cannot transition from STOW, but can from REVERSE_CLIMB (going up and down) and
        // READY_TO_CLIMB
        if (!RobotState.getInstance().getClimberState().equals(ClimberState.READY_TO_CLIMB)) {
          invalidStateTransition.setText(
              "Cannot transition Climber from " + RobotState.getInstance().getClimberState().name() +  " to CLIMB.  Robot needs to be READY_TO_CLIMB before performing any climbing action.");
          break;
        } else { // otherwise transition is valid
          RobotState.getInstance().setClimberState(ClimberState.CLIMB);
          break;
        }
      }

    // if the goal state and the actual state are not equal, then there was an error
    invalidStateTransition.set(RobotState.getInstance().getClimberState() != goalState);
  }

  /** Handles the the climber's state when there is no climber input */
  public void handleNoInputState() {
    setGoalState(
        getGoalState() == ClimberState.STOW
            ? ClimberState.STOW // if the robot is not climbing, stay in STOW state
            : ClimberState
                .READY_TO_CLIMB); // if the robot is climbing, transition to READY_TO_CLIMB
  }

  /**
   * @return true if cage is detected by climber LaserCAN
   */
  public boolean isCageInClimber() {
    return inputs.laserCanMeasurement.getStatus() == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && inputs.laserCanMeasurement.getDistance()
            < (int) Math.floor(laserCanDetectThreshold.get());
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void zeroEncoder() {
    io.zeroEncoder();
  }

  // public void setAngle(double rads) {
  //   io.setAngle(rads);
  // }
}
