/*
 * CoralHandoffIOKrakenFOC.java
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

import org.team5924.frc2025.Constants;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIOKrakenFOC;

public class CoralHandoffIOKrakenFOC extends GenericRollerSystemIOKrakenFOC
    implements CoralHandoffIO {
  private static final int id = Constants.CORAL_HANDOFF_CAN_ID;
  private static final String bus = Constants.CORAL_HANDOFF_BUS;
  private static final int currentLimitAmps = Constants.CORAL_HANDOFF_CURRENT_LIMIT;
  private static final boolean invert = Constants.CORAL_HANDOFF_INVERT;
  private static final boolean brake = Constants.CORAL_HANDOFF_BRAKE;
  private static final double reduction = Constants.CORAL_HANDOFF_REDUCTION;

  public CoralHandoffIOKrakenFOC() {
    super(id, bus, currentLimitAmps, invert, brake, reduction);
  }
}
