/*
 * AlgaePivotIO.java
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

import org.littletonrobotics.junction.AutoLog;

public interface AlgaePivotIO {
  /** Creates a new AlgaePivotIO. */
  @AutoLog
  public static class AlgaePivotIOInputs {
    public boolean algaePivotMotorConnected = true;
    public double algaePivotPositionRads = 0.0;
    public double algaePivotVelocityRadsPerSec = 0.0;
    public double algaePivotAppliedVolts = 0.0;
    public double algaePivotSupplyCurrentAmps = 0.0;
    public double algaePivotTorqueCurrentAmps = 0.0;
    public double algaePivotTempCelsius = 0.0;

    public boolean algaePivotCANcoderConnected = true;
    public double algaePivotCANcoderAbsolutePositionRads = 0.0;
    public double algaePivotCANcoderRelativePositionRads = 0.0;
  }

  default void updateInputs(AlgaePivotIOInputs inputs) {}

  /** Run roller at volts */
  public default void setVoltage(double volts) {}

  public default void setPosition(double rads) {}

  public default void setSoftStopOn() {}

  public default void setSoftStopOff() {}

  /** Stop roller */
  default void stop() {}
}
