/*
 * AlgaeRollerIO.java
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

package org.team5924.frc2025.subsystems.rollers.Algae;

import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIO;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIO.GenericRollerSystemIOInputs;
import org.team5924.frc2025.util.LaserCAN_Measurement;

public interface AlgaeRollerIO extends GenericRollerSystemIO {

  @AutoLog
  abstract class AlgaeRollerIOInputs extends GenericRollerSystemIOInputs {
    // Algae LaserCAN
    public LaserCAN_Measurement algaeRollerLCMeasurement = new LaserCAN_Measurement();
    public boolean algaeRollerLCConnected = true;
  }

  /** Run roller at volts */
  default void runVolts(double volts) {}
}
