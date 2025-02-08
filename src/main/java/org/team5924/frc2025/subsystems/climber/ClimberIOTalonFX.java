/*
 * ClimberIOTalonFX.java
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

package org.team5924.frc2025.subsystems.climber;

import static edu.wpi.first.units.Units.Radians;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.util.LaserCAN_Measurement;
import org.team5924.frc2025.util.exceptions.SensorRuntimeException;

/** Add your docs here. */
public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  private final double reduction;

  private static final LaserCan laserCan = new LaserCan(Constants.CLIMBER_LASER_CAN_ID);

  private static final Alert laserCanDisconnectAlert =
      new Alert("Climber LaserCAN disconnected.", AlertType.kWarning);
  private static final Alert laserCanInvalidMeasure =
      new Alert("Climber LaserCAN grabbed invalid measurement. See logs.", AlertType.kWarning);

  public ClimberIOTalonFX() {
    reduction = Constants.SHOOTER_REDUCTION;
    talon = new TalonFX(Constants.SHOOTER_CAN_ID, Constants.SHOOTER_BUS);

    // Configure TalonFX
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        Constants.SHOOTER_INVERT
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode =
        Constants.SHOOTER_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = Constants.SHOOTER_CURRENT_LIMIT;
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
  public void updateInputs(ClimberIOInputs inputs) {
    try {
      inputs.laserCanMeasurement = LaserCAN_Measurement.fromLaserCAN(laserCan.getMeasurement());
      inputs.laserCanConnected = true;
      laserCanDisconnectAlert.set(false);
      laserCanInvalidMeasure.set(false);
    } catch (SensorRuntimeException e) {
      switch (e.getErrorType()) {
        case DISCONNECTED -> {
          inputs.laserCanConnected = false;
          laserCanDisconnectAlert.set(true);
        }
        case INVALID_DATA -> laserCanInvalidMeasure.set(true);
        default -> {
          if (Constants.ALLOW_ASSERTS) throw e;
          else System.err.println("FIX NOW: Unhandled SensorRuntimeException: " + e.getMessage());
        }
      }
    }

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
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setAngle(double rads) {
    if (rads < Constants.CLIMBER_MIN_RADS || rads > Constants.CLIMBER_MAX_RADS) {
      Logger.recordOutput(
          "Climber/InvalidAngle",
          "Cannot set climber angle to "
              + rads
              + " rads.  This value extends past the climber angle boundary.");
    }
    rads = Math.min(Constants.CLIMBER_MIN_RADS, Math.max(rads, Constants.CLIMBER_MAX_RADS));
    talon.setControl(positionOut.withPosition(Radians.of(rads)));
  }
}
