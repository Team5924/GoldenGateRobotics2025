/*
 * CoralHandoffIOSim.java
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

package org.team5924.frc2025.subsystems.rollers.handoff;

import edu.wpi.first.math.system.plant.DCMotor;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIOSim;

public class CoralHandoffIOSim extends GenericRollerSystemIOSim implements CoralHandoffIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
  private static final double reduction = Constants.CORAL_HANDOFF_REDUCTION;
  private static final double moi = Constants.CORAL_HANDOFF_SIM_MOI;

  public CoralHandoffIOSim() {
    super(motorModel, reduction, moi);
  }
}
