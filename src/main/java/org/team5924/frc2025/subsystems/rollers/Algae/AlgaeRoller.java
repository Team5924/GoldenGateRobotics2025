/*
 * AlgaeRoller.java
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

package org.team5924.frc2025.subsystems.rollers.algae;

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
public class AlgaeRoller extends GenericRollerSystem<AlgaeRoller.AlgaeRollerState> {
  @RequiredArgsConstructor
  @Getter
  public enum AlgaeRollerState implements VoltageState {
    NO_ALGAE(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/NoCoralVoltage", 0.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/NoCoralVoltage", 0.0)),
    INTAKING(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/IntakingVoltage", -12.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/IntakingVoltage", 12.0)),
    STORED_ALGAE(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/StoredVoltage", 0.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/StoredVoltage", 0.0)),
    SHOOTING(
        new LoggedTunableNumber("CoralInAndOut/LoadShootMotor/ShootingVoltage", 12.0),
        new LoggedTunableNumber("CoralInAndOut/HandoffMotor/ShootingVoltage", 0.0));

    private final DoubleSupplier voltageSupplier;
    private final DoubleSupplier handoffVoltage;
  }

  private AlgaeRollerState goalState = AlgaeRollerState.NO_ALGAE;

  protected final AlgaeRollerIOInputsAutoLogged algaeRollerInputs =
      new AlgaeRollerIOInputsAutoLogged();

  private static final LoggedTunableNumber algaeDetectThreshold =
      new LoggedTunableNumber("CoralInAndOutKrakenFOC/IntakeLaserCAN/DetectThreshold", 20);

  public AlgaeRoller(AlgaeRollerIO io) {
    super("AlgaeRoller", io);
  }

  @Override
  public void periodic() {
    ((AlgaeRollerIO) io).runVolts(goalState.getVoltageSupplier().getAsDouble());
    super.periodic();
  }

  public void setGoalState(AlgaeRollerState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case NO_ALGAE -> RobotState.getInstance().setAlgaeRollerState(AlgaeRollerState.NO_ALGAE);
      case INTAKING -> RobotState.getInstance().setAlgaeRollerState(AlgaeRollerState.INTAKING);
      case STORED_ALGAE ->
          RobotState.getInstance().setAlgaeRollerState(AlgaeRollerState.STORED_ALGAE);
      case SHOOTING -> RobotState.getInstance().setAlgaeRollerState(AlgaeRollerState.SHOOTING);
    }
  }

  /**
   * @return true if coral is detected by intake LaserCAN
   */
  public boolean hasAlgae() {
    return algaeRollerInputs.algaeRollerLCMeasurement.getDistance()
        < (int) Math.floor(algaeDetectThreshold.get());
  }

  /**
   * @return true if coral is detected by shooter LaserCAN
   */
}
