/*
 * ClimberIO.java
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

import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.util.LaserCAN_Measurement;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean motorConnected = true;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    // Climber LaserCAN
    public LaserCAN_Measurement laserCanMeasurement = new LaserCAN_Measurement();
    public boolean laserCanConnected = true;
  }

  /**
 * Refreshes the climber's sensor and status readings.
 * <p>
 * The default implementation performs no action. Implementations should override this method to update the
 * provided {@code ClimberIOInputs} instance with the latest hardware data including motor connection status,
 * position, velocity, applied voltage, supply and torque currents, temperature, and laser measurement details.
 *
 * @param inputs the {@link ClimberIOInputs} instance to update with current data
 */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
 * Drives the motor with the specified voltage.
 *
 * Provide a voltage value (in volts) to control the motor's output. Positive values
 * result in forward motion, while negative values yield reverse operation. The
 * default implementation is a no-op and must be overridden to interact with hardware.
 *
 * @param volts the voltage (in volts) to apply to the motor
 */
  public default void runVolts(double volts) {}

  /**
 * Configures the climber to move to the specified target angle in radians.
 *
 * The target angle must be within the inclusive range defined by
 * {@link Constants#CLIMBER_MIN_RADS} and {@link Constants#CLIMBER_MAX_RADS}.
 *
 * @param rads target angle in radians
 * @throws IllegalArgumentException if {@code rads} is less than {@link Constants#CLIMBER_MIN_RADS} or greater than {@link Constants#CLIMBER_MAX_RADS}
 */
  public default void setAngle(double rads) {}

  /**
 * Stops the climber motor.
 *
 * <p>Halts motor operation by reducing the applied voltage to zero. This default implementation performs no
 * action and should be overridden by hardware-specific implementations to ensure proper motor shutdown behavior.
 */
  default void stop() {}
}
