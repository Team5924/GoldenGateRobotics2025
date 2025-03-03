/*
 * SimpleClimber.java
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

package org.team5924.frc2025.subsystems;

import static org.team5924.frc2025.Constants.CLIMBER_INVERT;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;

public class SimpleClimber extends SubsystemBase {
  private final TalonFX motor;
  private SimpleClimberState currentState = SimpleClimberState.STOW;

  public SimpleClimber(TalonFX motor) {
    this.motor = motor;

    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(CLIMBER_INVERT)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withFeedback(
                    new FeedbackConfigs().withSensorToMechanismRatio(Constants.CLIMBER_REDUCTION))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.CLIMBER_CURRENT_LIMIT)
                        .withSupplyCurrentLimitEnable(true)));
  }

  public SimpleClimberState getSimpleClimberState() {
    return currentState;
  }

  public enum SimpleClimberState {
    STOW(0, 0),
    LINEUP(12, 50 + 90 - 35),
    CLIMB(12, 50 + 90 + 90 + 35);

    public final double forwardVoltage;
    public final double wantedPositionDeg;

    SimpleClimberState(double forwardVoltage, double wantedPositionDeg) {
      this.forwardVoltage = forwardVoltage;
      this.wantedPositionDeg = wantedPositionDeg;
    }
  }

  @Override
  public void periodic() {
    var currentPositionDegrees = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());

    if (currentPositionDegrees < currentState.wantedPositionDeg) {
      motor.setVoltage(currentState.forwardVoltage);
    } else {
      motor.disable();
    }

    Logger.recordOutput("DEBUG/Climber/PositionDeg", currentPositionDegrees);
    Logger.recordOutput("DEBUG/Climber/WantedPosition", currentState.wantedPositionDeg);
    Logger.recordOutput("DEBUG/Climber/CurrentStateVoltage", currentState.forwardVoltage);
    Logger.recordOutput("DEBUG/Climber/CurrentState", currentState);
  }

  public void zeroEncoder() {
    motor.setPosition(0);
  }

  public void setState(SimpleClimberState newGoal) {
    currentState = newGoal;
  }
}
