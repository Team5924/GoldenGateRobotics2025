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
 * If this file has been seperated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.RobotState;

public class Climber extends SubsystemBase {
  @RequiredArgsConstructor
  @Getter
  public enum ClimberState {
    CLIMB,
    STOW,
    READY_TO_CLIMB,
    MOVING
  }

  private ClimberState goalState = ClimberState.STOW;

  @Getter private final ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void setGoalState(ClimberState goalState) {
    switch (goalState) {
      case CLIMB -> RobotState.getInstance().setClimberState(ClimberState.CLIMB);
      case STOW -> RobotState.getInstance().setClimberState(ClimberState.STOW);
      case READY_TO_CLIMB -> RobotState.getInstance().setClimberState(ClimberState.READY_TO_CLIMB);
      case MOVING -> RobotState.getInstance().setClimberState(ClimberState.MOVING);
    }
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void setAngle(double rads) {
    io.setAngle(rads);
  }
}
