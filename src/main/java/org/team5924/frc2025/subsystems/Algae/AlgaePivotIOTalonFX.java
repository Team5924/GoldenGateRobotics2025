/*
 * AlgaePivotIOTalonFX.java
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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.team5924.frc2025.Constants;

/** Add your docs here. */
public class AlgaePivotIOTalonFX implements AlgaePivotIO {
  private final TalonFX algaePivotTalon;

  private final StatusSignal<Angle> algaePivotPosition;
  private final StatusSignal<AngularVelocity> algaePivotVelocity;
  private final StatusSignal<Voltage> algaePivotAppliedVolts;
  private final StatusSignal<Current> algaePivotSupplyCurrent;
  private final StatusSignal<Current> algaePivotTorqueCurrent;
  private final StatusSignal<Temperature> algaePivotTempCelsius;

  private final VoltageOut voltageControl =
      new VoltageOut(0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final PositionVoltage positionControl =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public AlgaePivotIOTalonFX() {
    algaePivotTalon = new TalonFX(Constants.ALGAE_PIVOT_TALON_ID);

    // General config
    final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Feedback.SensorToMechanismRatio = Constants.MOTOR_TO_ALGAE_PIVOT_REDUCTION;

    final Slot0Configs controllerConfig = new Slot0Configs();
    controllerConfig.kP = 0;
    controllerConfig.kI = 0;
    controllerConfig.kD = 0;
    controllerConfig.kS = 0;

    algaePivotTalon.getConfigurator().apply(talonConfig, 1.0);
    algaePivotTalon.getConfigurator().apply(controllerConfig, 1.0);

    algaePivotPosition = algaePivotTalon.getPosition();
    algaePivotVelocity = algaePivotTalon.getVelocity();
    algaePivotAppliedVolts = algaePivotTalon.getMotorVoltage();
    algaePivotSupplyCurrent = algaePivotTalon.getSupplyCurrent();
    algaePivotTorqueCurrent = algaePivotTalon.getTorqueCurrent();
    algaePivotTempCelsius = algaePivotTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        algaePivotPosition,
        algaePivotVelocity,
        algaePivotAppliedVolts,
        algaePivotSupplyCurrent,
        algaePivotTorqueCurrent,
        algaePivotTempCelsius);
  }

  @Override
  public void updateInputs(AlgaePivotIOInputs inputs) {
    inputs.algaePivotMotorConnected =
        BaseStatusSignal.refreshAll(
                algaePivotPosition,
                algaePivotVelocity,
                algaePivotAppliedVolts,
                algaePivotSupplyCurrent,
                algaePivotTorqueCurrent,
                algaePivotTempCelsius)
            .isOK();

    inputs.algaePivotPositionRads = algaePivotPosition.getValue().in(Radians);
    inputs.algaePivotVelocityRadsPerSec = algaePivotVelocity.getValue().in(RadiansPerSecond);
    inputs.algaePivotAppliedVolts = algaePivotAppliedVolts.getValue().in(Volts);
    inputs.algaePivotSupplyCurrentAmps = algaePivotSupplyCurrent.getValue().in(Amps);
    inputs.algaePivotTorqueCurrentAmps = algaePivotTorqueCurrent.getValue().in(Amps);
    inputs.algaePivotTempCelsius = algaePivotTempCelsius.getValue().in(Celsius);
  }

  @Override
  public void setVoltage(double volts) {
    algaePivotTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setPosition(double rads) {
    algaePivotTalon.setControl(positionControl.withPosition(rads));
  }
}
