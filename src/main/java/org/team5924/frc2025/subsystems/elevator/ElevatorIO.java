/*
 * ElevatorIO.java
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

package org.team5924.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** TODO: Add documentation. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean leftMotorConnected = true;
    public boolean rightMotorConnected = true;

    public double leftPositionRads = 0.0;
    public double leftVelocityRadsPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTorqueCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    public double rightPositionRads = 0.0;
    public double rightVelocityRadsPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTorqueCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;

    public double posMeters = 0.0;
    public double velMetersPerSecond = 0.0;

    public double motionMagicVelocityTarget = 0.0;
    public double motionMagicPositionTarget = 0.0;

    public double setpointMeters = 0.0;

    public double acceleration = 0.0;

    public boolean minSoftStop = false;
    public boolean maxSoftStop = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void periodicUpdates() {}

  public default void setHeight(double heightMeters) {}

  public default void setVoltage(double volts) {}

  public default void setSoftStopOn() {}

  public default void setSoftStopOff() {}
}
