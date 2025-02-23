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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.subsystems.elevator.Elevator.ElevatorState;
import org.team5924.frc2025.subsystems.pivot.AlgaePivot.AlgaePivotState;
import org.team5924.frc2025.util.LoggedTunableNumber;

@Setter
@Getter
public class Climber extends SubsystemBase {
  public enum ClimberState {
    // Pulling onto cage, lifting robot
    CLIMB(new LoggedTunableNumber("Climber/ClimbingVoltage", 12.0)),

    // Default state, will be here most of the match
    STOW(new LoggedTunableNumber("Climber/StowVoltage", 0.0)),

    // Ready to climb
    READY_TO_CLIMB(new LoggedTunableNumber("Climber/ReadyToClimbVoltage", 0.0)),

    // Lowering robot
    REVERSE_CLIMB(new LoggedTunableNumber("Climber/ReverseClimbingVoltage", -12.0));

    private final LoggedTunableNumber volts;

    ClimberState(LoggedTunableNumber volts) {
      this.volts = volts;
    }
  }

  // 1 = up, -1 = down
  // private double voltageMultiplier = 1;

  private ClimberState goalState = ClimberState.STOW;
  private ClimberState lastState;

  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();

  private final ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private static final LoggedTunableNumber laserCanDetectThreshold =
      new LoggedTunableNumber("Climber/LaserCAN/DetectThreshold", 20);

  public Climber(ClimberIO io) {
    this.io = io;

    disconnected = new Alert("Climber disconnected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    disconnected.set(!inputs.motorConnected);

    // If the robot's state is STOW && the cage is within range && elevator + algae pivot are both
    // stow,
    // then set the robot's state to READY_TO_CLIMB
    if (getGoalState() == ClimberState.STOW
        && isCageInClimber()
        && RobotState.getInstance().getElevatorState() == ElevatorState.INTAKE
        && RobotState.getInstance().getAlgaePivotState() == AlgaePivotState.INTAKE_FLOOR) {
      setGoalState(ClimberState.READY_TO_CLIMB);
    }

    if (getGoalState() != lastState) {
      stateTimer.reset();
      lastState = getGoalState();
    }

    io.runVolts(goalState.volts.getAsDouble());
    Logger.recordOutput("Climber/Climber Goal", goalState.toString());
  }

  /**
   * Sets the goal state of the climber.
   *
   * @param goalState the new goal state
   */
  public void setGoalState(ClimberState goalState) {

    this.goalState = goalState;
    switch (goalState) {
      case CLIMB:
        if (RobotState.getInstance().getClimberState().equals(ClimberState.STOW)) {
          DriverStation.reportError(
              "Cannot transition Climber from STOW to "
                  + goalState.name()
                  + ".  Robot needs to be READY_TO_CLIMB before performing any climbing action",
              new StackTraceElement[] {
                new StackTraceElement("Climber", "setGoalState", "Climber", 106)
              });
          break;
        } else if (RobotState.getInstance().getClimberState().equals(ClimberState.READY_TO_CLIMB)) {
          RobotState.getInstance().setClimberState(ClimberState.CLIMB);
          break;
        }

      case STOW:
        RobotState.getInstance().setClimberState(ClimberState.STOW);
        break;
      case READY_TO_CLIMB:
        RobotState.getInstance().setClimberState(ClimberState.READY_TO_CLIMB);
        break;

      case REVERSE_CLIMB:
        if (RobotState.getInstance().getClimberState().equals(ClimberState.STOW)) {
          DriverStation.reportError(
              "Cannot transition Climber from STOW to "
                  + goalState.name()
                  + ".  Robot needs to be READY_TO_CLIMB before performing any climbing action",
              new StackTraceElement[] {
                new StackTraceElement("Climber", "setGoalState", "Climber", 106)
              });
          break;
        } else if (RobotState.getInstance().getClimberState().equals(ClimberState.READY_TO_CLIMB)) {
          RobotState.getInstance().setClimberState(ClimberState.REVERSE_CLIMB);
          break;
        }
    }
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

  public void setAngle(double rads) {
    io.setAngle(rads);
  }
}
