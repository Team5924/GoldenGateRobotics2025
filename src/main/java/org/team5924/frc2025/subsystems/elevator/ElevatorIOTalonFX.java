/*
 * ElevatorIOTalonFX.java
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

package org.team5924.frc2025.subsystems.elevator;

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

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> leftTorqueCurrent;
  private final StatusSignal<Temperature> leftTempCelsius;
  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Current> rightTorqueCurrent;
  private final StatusSignal<Temperature> rightTempCelsius;

  private final VoltageOut voltageControl =
      new VoltageOut(0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final PositionVoltage positionControl =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public ElevatorIOTalonFX() {
    leftTalon = new TalonFX(0);
    rightTalon = new TalonFX(0);

    // General config
    final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Feedback.SensorToMechanismRatio = 4;

    final Slot0Configs controllerConfig = new Slot0Configs();
    controllerConfig.kP = 0;
    controllerConfig.kI = 0;
    controllerConfig.kD = 0;
    controllerConfig.kS = 0;

    leftTalon.getConfigurator().apply(talonConfig, 1.0);
    rightTalon.getConfigurator().apply(talonConfig, 1.0);
    leftTalon.getConfigurator().apply(controllerConfig, 1.0);
    rightTalon.getConfigurator().apply(controllerConfig, 1.0);

    leftPosition = leftTalon.getPosition();
    leftVelocity = leftTalon.getVelocity();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftSupplyCurrent = leftTalon.getSupplyCurrent();
    leftTorqueCurrent = leftTalon.getTorqueCurrent();
    leftTempCelsius = leftTalon.getDeviceTemp();

    rightPosition = rightTalon.getPosition();
    rightVelocity = rightTalon.getVelocity();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightSupplyCurrent = rightTalon.getSupplyCurrent();
    rightTorqueCurrent = rightTalon.getTorqueCurrent();
    rightTempCelsius = rightTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftSupplyCurrent,
        leftTorqueCurrent,
        leftTempCelsius,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightSupplyCurrent,
        rightTorqueCurrent,
        rightTempCelsius);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorConnected =
        BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftAppliedVolts,
                leftSupplyCurrent,
                leftTorqueCurrent,
                leftTempCelsius)
            .isOK();
    inputs.rightMotorConnected =
        BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rightTorqueCurrent,
                rightTempCelsius)
            .isOK();

    inputs.leftPositionRads = leftPosition.getValue().in(Radians);
    inputs.leftVelocityRadsPerSec = leftVelocity.getValue().in(RadiansPerSecond);
    inputs.leftAppliedVolts = leftAppliedVolts.getValue().in(Volts);
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValue().in(Amps);
    inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValue().in(Amps);
    inputs.leftTempCelsius = leftTempCelsius.getValue().in(Celsius);

    inputs.rightPositionRads = rightPosition.getValue().in(Radians);
    inputs.rightVelocityRadsPerSec = rightVelocity.getValue().in(RadiansPerSecond);
    inputs.rightAppliedVolts = rightAppliedVolts.getValue().in(Volts);
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValue().in(Amps);
    inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValue().in(Amps);
    inputs.rightTempCelsius = rightTempCelsius.getValue().in(Celsius);
  }

  @Override
  public void setPosition(double rads) {
    leftTalon.setControl(positionControl.withPosition(Radians.of(rads)));
  }
}
