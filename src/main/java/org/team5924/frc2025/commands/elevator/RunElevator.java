/*
 * RunElevator.java
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

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.subsystems.elevator.Elevator;
import org.team5924.frc2025.subsystems.elevator.Elevator.ElevatorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevator extends Command {
  private final Elevator elevator;
  private final DoubleSupplier joystickY;

  /** Creates a new RunElevator. */
  public RunElevator(Elevator elevator, DoubleSupplier joystickY) {
    this.elevator = elevator;
    this.joystickY = joystickY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setSoftStopOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (RobotState.getInstance().getElevatorState()) {
      case MOVING -> {
        if (elevator.isAtSetpoint()) {
          RobotState.getInstance().setElevatorState(elevator.getGoalState());
        }
      }
      case MANUAL -> {
        elevator.setVoltage(
            Math.abs(joystickY.getAsDouble()) > 0.05 ? (-joystickY.getAsDouble() * 6) : 0.0);
      }
      case INTAKE -> elevator.setGoalState(ElevatorState.INTAKE);
      case L1 -> elevator.setGoalState(ElevatorState.L1);
      case L2 -> elevator.setGoalState(ElevatorState.L2);
      case L3 -> elevator.setGoalState(ElevatorState.L3);
      case L4 -> elevator.setGoalState(ElevatorState.L4);
      default -> {}
    }
    // elevator.setVoltage(-joystickY.getAsDouble() * 5);
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
