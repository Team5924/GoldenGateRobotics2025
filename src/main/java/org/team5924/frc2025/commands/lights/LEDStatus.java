/*
 * LEDStatus.java
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

package org.team5924.frc2025.commands.lights;

import edu.wpi.first.wpilibj2.command.Command;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.subsystems.lights.Lights;

public class LEDStatus extends Command {
  /** Creates a new LEDstatus. */
  private Lights leds;

  double limeErrorTolerance = 1.5; // in degrees

  public LEDStatus(Lights leds) {
    this.leds = leds;
    // addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (RobotState.getInstance().getElevatorState()) {
      case L3 -> Lights.LEDSegment.MainStrip.setColor(Lights.blue);
      default -> Lights.LEDSegment.MainStrip.disableLEDs();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
