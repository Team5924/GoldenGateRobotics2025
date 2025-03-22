/*
 * AlgaeRollerIOKrakenFOC.java
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

package org.team5924.frc2025.subsystems.rollers.algae;

import org.team5924.frc2025.Constants;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIOKrakenFOC;

public class AlgaeRollerIOKrakenFOC extends GenericRollerSystemIOKrakenFOC
    implements AlgaeRollerIO {

  private static final int AlgaeRollerId = Constants.ALGAE_TALON_ID;
  private static final String bus = Constants.ALGAE_BUS;
  private static final int currentLimitAmps = Constants.ALGAE_CURRENT_LIMIT;
  private static final boolean invert = Constants.ALGAE_INVERT;
  private static final boolean loadShootBrake = Constants.ALGAE_BRAKE;
  private static final double reduction = Constants.ALGAE_REDUCTION;

  public AlgaeRollerIOKrakenFOC() {
    super(AlgaeRollerId, bus, currentLimitAmps, invert, loadShootBrake, reduction);
  }

  public void updateInputs(AlgaeRollerIOInputs inputs) {

    super.updateInputs(inputs);
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
  }
}
