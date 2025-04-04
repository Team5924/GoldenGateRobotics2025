/*
 * GenericRollerSystemIO.java
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

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerSystemIO {

  @AutoLog
  abstract class GenericRollerSystemIOInputs {

    /** Whether the motor is detected and sending status signals. */
    public boolean motorConnected = true;

    /** Current position of the motor in radians. */
    public double positionRads = 0.0;

    /** Current velocity of the motor in radians per second. */
    public double velocityRadsPerSec = 0.0;

    /** Voltage applied to the motor in volts. */
    public double appliedVoltage = 0.0;

    /** Motor supply current in amps. */
    public double supplyCurrentAmps = 0.0;

    /** Motor torque current in amps. */
    public double torqueCurrentAmps = 0.0;

    /** Motor temperature in Celsius. */
    public double tempCelsius = 0.0;
  }

  default void updateInputs(GenericRollerSystemIOInputs inputs) {}

  /** Run roller at volts */
  default void runVolts(double volts) {}

  /** Stop roller */
  default void stop() {}
}
