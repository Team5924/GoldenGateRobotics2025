/*
 * AlgaePivot.java
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

package org.team5924.frc2025.subsystems.Algae;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.util.LoggedTunableNumber;

public class AlgaePivot extends SubsystemBase {

  private final AlgaePivotIO io;

  public LoggedTunableNumber AlgaePivotTolerance =
      new LoggedTunableNumber("AlgaePivotToleranceRads", .02);
  private final AlgaePivotIOInputsAutoLogged inputs = new AlgaePivotIOInputsAutoLogged();

  public enum AlgaePivotState {
    INTAKE_HIGH(new LoggedTunableNumber("AlgaePivotIntakeHighRads", 0)),
    INTAKE_LOW(new LoggedTunableNumber("AlgaePivotIntakeLowRads", 0)),
    INTAKE_FLOOR(new LoggedTunableNumber("AlgaePivotIntakeFloorRads", 0.5)),
    NET(new LoggedTunableNumber("AlgaePivotIntakeNetRads", 1)),
    MANUAL(new LoggedTunableNumber("AlgaePivotManualRads", -1)),
    MOVING(new LoggedTunableNumber("AlgaePivotMovingRads", -1)),
    HOLDING(new LoggedTunableNumber("AlgaePivotHoldingRads", -1));

    private final LoggedTunableNumber rads;

    AlgaePivotState(LoggedTunableNumber rads) {
      this.rads = rads;
    }
  }

  @Getter private AlgaePivotState goalState;

  private final Alert AlgaePivotMotorDisconnected;

  /** Creates a new AlgaePivot. */
  public AlgaePivot(AlgaePivotIO io) {
    this.io = io;
    this.goalState = AlgaePivotState.INTAKE_FLOOR;
    this.AlgaePivotMotorDisconnected =
        new Alert("Left elevator motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("AlgaePivot", inputs);

    Logger.recordOutput("AlgaePivot/GoalState", goalState.toString());
    Logger.recordOutput("AlgaePivot/TargetRads", goalState.rads);

    AlgaePivotMotorDisconnected.set(!inputs.algaePivotMotorConnected);
  }

  private double getAlgaePivotPositionRads() {
    return inputs.algaePivotPositionRads / Constants.MOTOR_TO_ALGAE_PIVOT_REDUCTION;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getAlgaePivotPositionRads() - this.goalState.rads.getAsDouble())
        < AlgaePivotTolerance.getAsDouble();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setGoalState(AlgaePivotState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case MANUAL:
        RobotState.getInstance().setAlgaePivotState(AlgaePivotState.MANUAL);
        break;
      case MOVING:
        DriverStation.reportError("Invalid goal AlgaePivotState!", null);
        break;
      default:
        RobotState.getInstance().setAlgaePivotState(AlgaePivotState.MOVING);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }
  }
}
