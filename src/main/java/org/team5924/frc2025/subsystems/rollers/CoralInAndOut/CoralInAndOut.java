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
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystem;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystem.VoltageState;
import org.team5924.frc2025.util.LoggedTunableNumber;

@Setter
@Getter
public class CoralInAndOut extends GenericRollerSystem<CoralInAndOut.CoralState> {
  @RequiredArgsConstructor
  @Getter
  public enum CoralState implements VoltageState {
    NO_CORAL(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/NoCoralVoltage", 0.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/NoCoralVoltage", 0.0)),
    INTAKING(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/IntakingVoltage", -12.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/IntakingVoltage", 12.0)),
    STORED_CORAL(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/StoredVoltage", 0.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/StoredVoltage", 0.0)),
    SHOOTING(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/ShootingVoltage", 12.0),
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

  public CoralInAndOut(CoralInAndOutIO io) {
    super("CoralInAndOut", io);
  }

  @Override
  public void periodic() {
    ((CoralInAndOutIO) io)
        .runVolts(
            goalState.getVoltageSupplier().getAsDouble(),
            goalState.getHandoffVoltage().getAsDouble());
    super.periodic();
  }

  public void setGoalState(CoralState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case NO_CORAL -> RobotState.getInstance().setCoralInAndOutState(CoralState.NO_CORAL);
      case INTAKING -> RobotState.getInstance().setCoralInAndOutState(CoralState.INTAKING);
      case STORED_CORAL -> RobotState.getInstance().setCoralInAndOutState(CoralState.STORED_CORAL);
      case SHOOTING -> RobotState.getInstance().setCoralInAndOutState(CoralState.SHOOTING);
      case SPIT_BACK -> RobotState.getInstance().setCoralInAndOutState(CoralState.SPIT_BACK);
    }
  }

  /**
   * @return true if coral is detected by intake LaserCAN
   */
  public boolean isCoralInIntake() {
    return coralInputs.intakeLCMeasurement.getDistance()
        < (int) Math.floor(intakeDetectThreshold.get());
  }

  /**
   * @return true if coral is detected by shooter LaserCAN
   */
  public boolean isCoralInShooter() {
    return coralInputs.shooterLCMeasurement.getDistance()
        < (int) Math.floor(shooterDetectThreshold.get());
  }
}
