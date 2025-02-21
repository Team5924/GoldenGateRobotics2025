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

  /**
   * Instantiates and configures the TalonFX motor controller for the climber subsystem.
   *
   * <p>Initializes the reduction factor using <code>Constants.CLIMBER_REDUCTION</code> and creates the
   * TalonFX instance with the CAN ID and bus specified in <code>Constants</code>. Configures the motor
   * controller by setting inversion (clockwise positive if <code>Constants.CLIMBER_INVERT</code> is true,
   * otherwise counter-clockwise positive) and the neutral mode (brake if <code>Constants.CLIMBER_BRAKE</code>
   * is true, otherwise coast). Applies the supply current limit from <code>Constants.CLIMBER_CURRENT_LIMIT</code>
   * and enables current limiting.</p>
   *
   * <p>Retrieves status signals for position, velocity, applied voltage, supply current, torque current, and
   * device temperature, setting an update frequency of 50 Hz for all. Optimizes bus utilization by disabling
   * unused status signals.</p>
   */
  public ClimberIOTalonFX() {
    reduction = Constants.CLIMBER_REDUCTION;
    talon = new TalonFX(Constants.CLIMBER_CAN_ID, Constants.CLIMBER_BUS);

    // Configure TalonFX
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        Constants.CLIMBER_INVERT
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode =
        Constants.CLIMBER_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = Constants.CLIMBER_CURRENT_LIMIT;
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

  /**
   * Refreshes sensor and motor status inputs for the climber subsystem.
   *
   * <p>
   * Retrieves a measurement from the laser CAN sensor and updates the corresponding fields in the provided
   * {@code ClimberIOInputs} instance. On a successful read, the laser measurement is converted via
   * {@code LaserCAN_Measurement.fromLaserCAN} and the laser connection flag is set to true while clearing any
   * disconnection or invalid measurement alerts.
   * </p>
   *
   * <p>
   * If a {@code SensorRuntimeException} is caught during the laser sensor read:
   * <ul>
   *   <li>If the error type is {@code DISCONNECTED}, the laser connection flag is set to false and the disconnection alert is activated.</li>
   *   <li>If the error type is {@code INVALID_DATA}, the invalid measurement alert is activated.</li>
   *   <li>For other error types, the exception is either rethrown (when assertions are enabled) or an error is logged.</li>
   * </ul>
   * </p>
   *
   * <p>
   * Additionally, it refreshes all motor status signals (position, velocity, applied voltage, supply current, torque current,
   * and temperature) using {@code BaseStatusSignal.refreshAll}. The motor's position and velocity are converted from rotations
   * to radians and adjusted by the reduction factor before being updated in {@code inputs}. The motor connection status is
   * determined based on the successful refresh of these signals.
   * </p>
   *
   * @param inputs the {@code ClimberIOInputs} object to be updated with current sensor measurements and motor status data;
   *               must not be {@code null}
   * @throws SensorRuntimeException if an unexpected sensor error occurs and assertions are enabled
   */
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

  /**
   * Commands the TalonFX motor controller to output a specified voltage.
   *
   * <p>Converts the desired voltage into a control signal using the configured voltage output
   * mechanism and applies it to the motor. This method is used primarily for open-loop control of the motor.
   *
   * @param volts the voltage (in volts) to be applied to the motor
   */
  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  /**
   * Sets the climber's target angle in radians.
   *
   * Validates that the specified angle lies within the bounds defined by
   * {@link Constants#CLIMBER_MIN_RADS} and {@link Constants#CLIMBER_MAX_RADS}. If the provided
   * angle is outside these limits, an error is logged and an {@link IllegalArgumentException}
   * is thrown. When the angle is valid, the method updates the motor controller's control output
   * to achieve the desired position.
   *
   * @param rads the target angle in radians
   * @throws IllegalArgumentException if {@code rads} is not within the permitted range
   */
  @Override
  public void setAngle(double rads) throws IllegalArgumentException {
    if (rads < Constants.CLIMBER_MIN_RADS || rads > Constants.CLIMBER_MAX_RADS) {
      String message =
          "Cannot set climber angle to "
              + rads
              + " rads.  This value extends past the climber angle boundary.";
      Logger.recordOutput("Climber/InvalidAngle", message);
      throw new IllegalArgumentException(message);
    }
    rads = Math.min(Constants.CLIMBER_MIN_RADS, Math.max(rads, Constants.CLIMBER_MAX_RADS));
    talon.setControl(positionOut.withPosition(Radians.of(rads)));
  }
}
