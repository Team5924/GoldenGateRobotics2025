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
 * If this file has been seperated from the original project, you should have
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
public class CoralInAndOut extends GenericRollerSystem<CoralInAndOut.State> {
  @RequiredArgsConstructor
  @Getter
  public enum State implements VoltageState {
    LOADING(new LoggedTunableNumber("CoralInAndOut/LoadingVoltage", 12.0)),
    SHOOTING(new LoggedTunableNumber("CoralInAndOut/ShootingVoltage", 12.0)),
    EMPTY(new LoggedTunableNumber("CoralInAndOut/EmptyVoltage", 0.0)),

    // Used for L1 Scoring
    HOLDING(new LoggedTunableNumber("CoralInAndOut/HoldingVoltage", 0.0)),
    SPIT_BACK(new LoggedTunableNumber("CoralInAndOut/SpitBackVoltage", -12.0));

    private final DoubleSupplier voltageSupplier;
  }

  private State state = State.EMPTY;

  public CoralInAndOut(CoralInAndOutIO io) {
    super("CoralInAndOut", io);
  }

  @Override
  public void periodic() {
    getIo().runVolts(state.getVoltageSupplier().getAsDouble());
    super.periodic();
  }

  public void setState(State newGoal) {
    if (this.state != newGoal) {
      this.state = newGoal;
      RobotState.getInstance().setCoralInAndOutGoal(newGoal);
    }
  }
}
