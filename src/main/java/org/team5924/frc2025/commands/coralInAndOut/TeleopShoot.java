/*
 * TeleopShoot.java
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

package org.team5924.frc2025.commands.coralInAndOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.subsystems.elevator.Elevator.ElevatorState;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOut;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOut.CoralState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopShoot extends Command {
  private final CoralInAndOut coralInAndOut;

  /** Creates a new RunShooterStateMachine. */
  public TeleopShoot(CoralInAndOut coralInAndOut) {
    this.coralInAndOut = coralInAndOut;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralInAndOut);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Command Initialize", "Yes");
    if (RobotState.getInstance().getElevatorState().equals(ElevatorState.L4)) {
      coralInAndOut.setGoalState(CoralState.SHOOTING_L4);
      SmartDashboard.putString("Elevator_state", "Shooting L4");
    } else {
      coralInAndOut.setGoalState(CoralState.SHOOTING_L2_AND_L3);
      SmartDashboard.putString("Elevator_state", "Shooting L2+3");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralInAndOut.setGoalState(CoralState.NO_CORAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
