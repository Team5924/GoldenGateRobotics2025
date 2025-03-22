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
    STOPPED(
        new LoggedTunableNumber("AlgaeRoller/NoAlgaeVoltage", 0.0),
        new LoggedTunableNumber("DummyValue", 0.0)),
    SPINNING(
        new LoggedTunableNumber("AlgaeRoller/IntakingVoltage", -12.0),
        new LoggedTunableNumber("DummyValue", 0.0));

    private final DoubleSupplier voltageSupplier;
    private final DoubleSupplier dummySupplier;
  }

  private AlgaeRollerState goalState = AlgaeRollerState.STOPPED;

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
      case STOPPED -> RobotState.getInstance().setAlgaeRollerState(AlgaeRollerState.STOPPED);
      case SPINNING -> RobotState.getInstance().setAlgaeRollerState(AlgaeRollerState.SPINNING);
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
