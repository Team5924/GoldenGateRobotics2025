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
 * If this file has been separated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025.subsystems.pivot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.util.Elastic.Notification;
import org.team5924.frc2025.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2025.util.LoggedTunableNumber;

public class AlgaePivot extends SubsystemBase {

  private final AlgaePivotIO io;

  public LoggedTunableNumber AlgaePivotTolerance =
      new LoggedTunableNumber("AlgaePivotToleranceRads", .02);
  private final AlgaePivotIOInputsAutoLogged inputs = new AlgaePivotIOInputsAutoLogged();

  public enum AlgaePivotState {
    DEPLOY(new LoggedTunableNumber("AlgaePivotIntakeHighRads", -13.5)),
    STOW(new LoggedTunableNumber("AlgaePivotIntakeLowRads", .1));

    private final LoggedTunableNumber rads;

    AlgaePivotState(LoggedTunableNumber rads) {
      this.rads = rads;
    }
  }

  @Getter private AlgaePivotState goalState;

  private final Alert AlgaePivotMotorDisconnected;

  private final Notification algaePivotMotorDisconnectedNotification;

  /** Creates a new AlgaePivot. */
  public AlgaePivot(AlgaePivotIO io) {
    this.io = io;
    this.goalState = AlgaePivotState.STOW;
    this.AlgaePivotMotorDisconnected =
        new Alert("Algae pivot motor disconnected!", Alert.AlertType.kWarning);

    algaePivotMotorDisconnectedNotification =
        new Notification(
            NotificationLevel.WARNING, "Algae Pivot Warning", "Algae Pivot motor disconnected!");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("AlgaePivot", inputs);

    Logger.recordOutput("AlgaePivot/GoalState", goalState.toString());
    Logger.recordOutput("AlgaePivot/TargetRads", goalState.rads);
    Logger.recordOutput("AlgaePivot/DifferenceRads", getDifferenceRads());
    Logger.recordOutput("AlgaePivot/IsAtSetpoint", isAtSetpoint());

    AlgaePivotMotorDisconnected.set(!inputs.algaePivotMotorConnected);

    if (!isAtSetpoint())
      io.setPosition(goalState.rads.getAsDouble());

    // if (!inputs.algaePivotMotorConnected)
    //   Elastic.sendNotification(algaePivotMotorDisconnectedNotification);
  }

  /**
   * @return the difference of the goal pivot rads and the current pivot rads
   */
  private double getDifferenceRads() {
    return goalState.rads.getAsDouble() - getAlgaePivotPositionRads();
  }

  private double getAlgaePivotPositionRads() {
    return inputs.algaePivotPositionRads / Constants.MOTOR_TO_ALGAE_PIVOT_REDUCTION;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getDifferenceRads()) <= AlgaePivotTolerance.getAsDouble();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setGoalState(AlgaePivotState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case STOW:
        RobotState.getInstance().setAlgaePivotState(AlgaePivotState.STOW);
        break;
      case DEPLOY:
        RobotState.getInstance().setAlgaePivotState(AlgaePivotState.DEPLOY);
        break;
    }
  }
}
