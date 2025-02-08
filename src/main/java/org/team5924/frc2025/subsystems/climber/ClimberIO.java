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

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void setAngle(double rads) {}

  default void stop() {}
}
