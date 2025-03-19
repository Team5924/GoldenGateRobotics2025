/*
 * CoralInAndOut.java
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

package org.team5924.frc2025.subsystems.rollers.CoralInAndOut;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.subsystems.elevator.Elevator.ElevatorState;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystem;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystem.VoltageState;
import org.team5924.frc2025.util.LoggedTunableNumber;

@Setter
@Getter
public class CoralInAndOut extends GenericRollerSystem<CoralInAndOut.CoralState> {
  @RequiredArgsConstructor
  @Getter
  /*
   * ALL LOGGEDTUNABLE DEFAULT VALUES ARE TERRIBLE CONSTANTS
   *
   */
  public enum CoralState implements VoltageState {
    NO_CORAL(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/NoCoralVoltage", 0.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/NoCoralVoltage", 0.0)),
    INTAKING(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/IntakingVoltage", -12.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/IntakingVoltage", 12.0)),
    STORED_CORAL_IN_INTAKE(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/StoredVoltage", 0.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/StoredVoltage", 0.0)),
    STORED_CORAL_IN_SHOOTER(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/StoredVoltage", 0.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/StoredVoltage", 0.0)),
    SHOOTING_L2_AND_L3(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/L2AndL3ShootingVoltage", 3.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/ShootingVoltage", 5.0)),
    SHOOTING_L4(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/L4ShootingVoltage", 6.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/ShootingVoltage", 5.0)),
    SHOOTING_L1(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/L1ShootingVoltage", 2.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/ShootingVoltage", 0.0)),
    SPIT_BACK(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/SpitBackVoltage", -12.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/SpitBackVoltage", -12.0));

    private final DoubleSupplier voltageSupplier;
    private final DoubleSupplier handoffVoltage;
  }

  private CoralState goalState = CoralState.NO_CORAL;

  protected final CoralInAndOutIOInputsAutoLogged coralInputs =
      new CoralInAndOutIOInputsAutoLogged();

  private static final LoggedTunableNumber intakeDetectThreshold =
      new LoggedTunableNumber("CoralInAndOutKrakenFOC/IntakeLaserCAN/DetectThreshold", 20);

  private static final LoggedTunableNumber shooterDetectThreshold =
      new LoggedTunableNumber("CoralInAndOutKrakenFOC/ShooterLaserCAN/DetectThreshold", 20);

  private static final LoggedTunableNumber exitDetectThreshold =
      new LoggedTunableNumber("CoralInAndOutKrakenFOC/ExitLaserCAN/DetectThreshold", 20);

  public CoralInAndOut(CoralInAndOutIO io) {
    super("CoralInAndOut", io);
  }

  public void updateCoralState() {
    if (isCoralInShooter()
        && !RobotState.getInstance().getCoralInAndOutState().equals(CoralState.SHOOTING_L1)
        && !RobotState.getInstance().getCoralInAndOutState().equals(CoralState.SHOOTING_L2_AND_L3)
        && !RobotState.getInstance().getCoralInAndOutState().equals(CoralState.SHOOTING_L4)) {
      setGoalState(CoralState.STORED_CORAL_IN_SHOOTER);
    } else if (!isCoralInShooter()
        && !RobotState.getInstance().getCoralInAndOutState().equals(CoralState.SHOOTING_L1)
        && !RobotState.getInstance().getCoralInAndOutState().equals(CoralState.SHOOTING_L2_AND_L3)
        && !RobotState.getInstance().getCoralInAndOutState().equals(CoralState.SHOOTING_L4)
        && !RobotState.getInstance().getCoralInAndOutState().equals(CoralState.INTAKING)
        && !RobotState.getInstance().getCoralInAndOutState().equals(CoralState.SPIT_BACK)) {
      setGoalState(CoralState.NO_CORAL);
    }
  }

  @Override
  public void periodic() {
    ((CoralInAndOutIO) io)
        .runVolts(
            goalState.getVoltageSupplier().getAsDouble(),
            goalState.getHandoffVoltage().getAsDouble());
    super.periodic();

    // update CoralState periodically
    // updateCoralState();

    Logger.recordOutput(
        "RobotState/Coral/InAndOutState", RobotState.getInstance().getCoralInAndOutState());
  }

  public void setGoalState(CoralState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case NO_CORAL -> RobotState.getInstance().setCoralInAndOutState(CoralState.NO_CORAL);
      case INTAKING -> RobotState.getInstance().setCoralInAndOutState(CoralState.INTAKING);
      case STORED_CORAL_IN_INTAKE ->
          RobotState.getInstance().setCoralInAndOutState(CoralState.STORED_CORAL_IN_INTAKE);
      case STORED_CORAL_IN_SHOOTER ->
          RobotState.getInstance().setCoralInAndOutState(CoralState.STORED_CORAL_IN_SHOOTER);
      case SHOOTING_L2_AND_L3 -> {
        if ((RobotState.getInstance().getElevatorState().equals(ElevatorState.L2)
                || RobotState.getInstance().getElevatorState().equals(ElevatorState.L3))
            && RobotState.getInstance()
                .getCoralInAndOutState()
                .equals(CoralState.STORED_CORAL_IN_SHOOTER)) {
          RobotState.getInstance().setCoralInAndOutState(CoralState.SHOOTING_L2_AND_L3);
        }
      }
      case SHOOTING_L4 -> {
        if (RobotState.getInstance().getElevatorState().equals(ElevatorState.L4)
            && RobotState.getInstance()
                .getCoralInAndOutState()
                .equals(CoralState.STORED_CORAL_IN_SHOOTER)) {
          RobotState.getInstance().setCoralInAndOutState(CoralState.SHOOTING_L4);
        }
      }
      case SHOOTING_L1 -> {
        if (RobotState.getInstance().getElevatorState().equals(ElevatorState.L1)
            && RobotState.getInstance()
                .getCoralInAndOutState()
                .equals(CoralState.STORED_CORAL_IN_SHOOTER)) {
          RobotState.getInstance().setCoralInAndOutState(CoralState.SHOOTING_L1);
        }
      }
      case SPIT_BACK -> RobotState.getInstance().setCoralInAndOutState(CoralState.SPIT_BACK);
    }
  }

  /**
   * @return true if coral is detected by intake LaserCAN
   */
  // public boolean isCoralInIntake() {
  //   return coralInputs.intakeLCMeasurement.getDistance()
  //       < (int) Math.floor(intakeDetectThreshold.get());
  // }

  /**
   * @return true if coral is detected by shooter LaserCAN
   */
  public boolean isCoralInShooter() {
    return coralInputs.shooterLCMeasurement.getDistance()
        < (int) Math.floor(shooterDetectThreshold.get());
  }

  public void handleStateFromElevatorState(ElevatorState elevatorState) {

    // Set the volatge of the Coral In and Out based on the elevator state
    switch (elevatorState) {
      case L1:
        setGoalState(CoralState.SHOOTING_L1);
        break;

      case L2:
      case L3:
        setGoalState(CoralState.SHOOTING_L1);
        break;

      case L4:
        setGoalState(CoralState.SHOOTING_L4);
        break;

      default:
        setGoalState(CoralState.SPIT_BACK);
        break;
    }
  }

  /**
   * @return true if coral is detected by exit LaserCAN
   */
  // public boolean hasCoralExited() {
  //   return coralInputs.exitLCMeasurement.getDistance()
  //       < (int) Math.floor(exitDetectThreshold.get());
  // }
}
