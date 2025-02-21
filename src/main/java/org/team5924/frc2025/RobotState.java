/*
 * RobotState.java
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

package org.team5924.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team5924.frc2025.subsystems.elevator.Elevator.ElevatorState;
import org.team5924.frc2025.subsystems.pivot.AlgaePivot.AlgaePivotState;
import org.team5924.frc2025.subsystems.rollers.Algae.AlgaeRoller.AlgaeRollerState;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOut.CoralState;

@Getter
public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose Estimation Members
  @AutoLogOutput(key = "RobotState/OdometryPose")
  private Pose2d odometryPose = new Pose2d();

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  private Pose2d estimatedPose = new Pose2d();

  @Getter @Setter private ElevatorState elevatorState = ElevatorState.MANUAL;

  /* ### Coral In and Out ### */
  @Getter @Setter private CoralState coralInAndOutState = CoralState.NO_CORAL;

  /* ### Algae Pivot ### */
  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/AlgaePivotState")
  private AlgaePivotState algaePivotState = AlgaePivotState.INTAKE_FLOOR;

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/AlgaeRollerState")
  private AlgaeRollerState algaeRollerState = AlgaeRollerState.NO_ALGAE;
}
