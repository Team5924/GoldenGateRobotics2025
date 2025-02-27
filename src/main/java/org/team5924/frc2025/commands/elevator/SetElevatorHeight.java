/*
 * SetElevatorHeight.java
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

package org.team5924.frc2025.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.team5924.frc2025.subsystems.elevator.Elevator;
import org.team5924.frc2025.subsystems.elevator.Elevator.ElevatorState;

public class SetElevatorHeight extends Command {
  private Elevator elevator;
  private ElevatorState goalState;
  private Timer timer;

  /** Creates a new SetElevatorHeight. */
  public SetElevatorHeight(Elevator elevator, ElevatorState goalState) {
    this.elevator = elevator;
    this.goalState = goalState;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setGoalState(goalState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(3);
  }
}
