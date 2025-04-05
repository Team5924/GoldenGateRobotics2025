/*
 * SetLEDColorCommand.java
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
import org.team5924.frc2025.subsystems.Lights.LEDSubsystem;

public class SetLEDColorCommand extends Command {
  private final LEDSubsystem led;
  private final int r, g, b;

  public SetLEDColorCommand(LEDSubsystem led, int r, int g, int b) {
    this.led = led;
    this.r = r;
    this.g = g;
    this.b = b;
    addRequirements(led);
  }

  @Override
  public void initialize() {
    led.setColor(r, g, b);
  }

  @Override
  public boolean isFinished() {
    return true; // run once and done
  }
}
