/*
 * GenericRollerSystemIOKrakenFOC.java
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

package org.team5924.frc2025.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class GenericRollerSystemIOKrakenFOC implements GenericRollerSystemIO {
  private final TalonFX talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final double reduction;

  public GenericRollerSystemIOKrakenFOC(
      int id, String bus, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    talon = new TalonFX(id, bus);

    // Configure TalonFX
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    // Get select status signals and set update frequency
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);

    // Disables status signals not called for update above
    talon.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
            .isOK();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reduction;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    if (Math.abs(volts) > 12.0) {
      throw new IllegalArgumentException("Voltage must be between -12.0 and 12.0");
    }

    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }
}
