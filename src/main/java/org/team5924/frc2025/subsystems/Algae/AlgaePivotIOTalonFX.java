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
 * If this file has been separated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025.subsystems.algae;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration.*;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.util.LoggedTunableNumber;

/** Add your docs here. */
public class AlgaePivotIOTalonFX implements AlgaePivotIO {
  private final TalonFX algaePivotTalon;
  private final CANcoder algaePivotCANcoder;

  private final StatusSignal<Angle> algaePivotCANcoderAbsolutePositionRotations;
  private final StatusSignal<Angle> algaePivotCANcoderRelativePositionRotations;
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

  LoggedTunableNumber algaePivotMotorkP = new LoggedTunableNumber("AlgaePivotMotorkP", 0);
  LoggedTunableNumber algaePivotMotorkI = new LoggedTunableNumber("AlgaePivotMotorkI", 0);
  LoggedTunableNumber algaePivotMotorkD = new LoggedTunableNumber("AlgaePivotMotorkD", 0);
  LoggedTunableNumber algaePivotMotorkS = new LoggedTunableNumber("AlgaePivotMotorkS", 0);
  LoggedTunableNumber algaePivotCANcoderMagnetOffsetRads =
      new LoggedTunableNumber("AlgaePivotCANcoderOffsetRads", 0);
  LoggedTunableNumber algaePivotSensorDiscontinuityPoint =
      new LoggedTunableNumber("AlgaePivotSensorDiscontinuityPoint", .5);

  public AlgaePivotIOTalonFX() {
    algaePivotTalon = new TalonFX(Constants.ALGAE_PIVOT_TALON_ID);
    algaePivotCANcoder = new CANcoder(Constants.ALGAE_PIVOT_CANCODER_ID);

    CANcoderConfiguration algaePivotCANcoderConfiguration = new CANcoderConfiguration();
    algaePivotCANcoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(
        algaePivotSensorDiscontinuityPoint.getAsDouble());
    algaePivotCANcoderConfiguration.MagnetSensor.SensorDirection =
        SensorDirectionValue.Clockwise_Positive;
    algaePivotCANcoderConfiguration.MagnetSensor.MagnetOffset =
        Units.radiansToRotations(algaePivotCANcoderMagnetOffsetRads.getAsDouble());
    algaePivotCANcoder.getConfigurator().apply(algaePivotCANcoderConfiguration, 1.0);

    // General config
    final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Feedback.SensorToMechanismRatio = Constants.MOTOR_TO_ALGAE_PIVOT_REDUCTION;

    final Slot0Configs controllerConfig = new Slot0Configs();
    controllerConfig.kP = algaePivotMotorkP.getAsDouble();
    controllerConfig.kI = algaePivotMotorkI.getAsDouble();
    controllerConfig.kD = algaePivotMotorkD.getAsDouble();
    controllerConfig.kS = algaePivotMotorkS.getAsDouble();

    algaePivotTalon.getConfigurator().apply(talonConfig, 1.0);
    algaePivotTalon.getConfigurator().apply(controllerConfig, 1.0);

    algaePivotCANcoderAbsolutePositionRotations = algaePivotCANcoder.getAbsolutePosition();
    algaePivotCANcoderRelativePositionRotations = algaePivotCANcoder.getPosition();

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

    BaseStatusSignal.setUpdateFrequencyForAll(
        500,
        algaePivotCANcoderAbsolutePositionRotations,
        algaePivotCANcoderRelativePositionRotations);
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
    inputs.algaePivotCANcoderAbsolutePositionRads =
        Units.rotationsToRadians(algaePivotCANcoderAbsolutePositionRotations.getValueAsDouble())
            - algaePivotCANcoderMagnetOffsetRads.getAsDouble();
    inputs.algaePivotCANcoderRelativePositionRads =
        Units.rotationsToRadians(algaePivotCANcoderRelativePositionRotations.getValueAsDouble())
            - algaePivotCANcoderMagnetOffsetRads.getAsDouble();
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
