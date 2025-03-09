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
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team5924.frc2025.subsystems.climber.Climber.ClimberState;
import org.team5924.frc2025.subsystems.elevator.Elevator.ElevatorState;
import org.team5924.frc2025.subsystems.pivot.AlgaePivot.AlgaePivotState;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOut.CoralState;
import org.team5924.frc2025.util.VisionFieldPoseEstimate;

@Getter
public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose Estimation Members
  @AutoLogOutput(key = "RobotState/OdometryPose")
  @Getter
  @Setter
  private Pose2d odometryPose = new Pose2d();

  /* Vision Pose */
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  @Getter
  @Setter
  private VisionFieldPoseEstimate estimatedPose = new VisionFieldPoseEstimate();

  /* ### Climber ### */
  @Setter
  @AutoLogOutput(key = "RobotState/ClimberState")
  private ClimberState climberState = ClimberState.STOW;

  @Getter @Setter private Rotation2d yawPosition = new Rotation2d();
  @Getter @Setter private double yawVelocityRadPerSec = 0.0;

  @Getter @Setter private ElevatorState elevatorState = ElevatorState.MANUAL;
  @Getter @Setter private double elevatorPositionMeters = 0;

  /* ### Coral In and Out ### */
  @Getter @Setter private CoralState coralInAndOutState = CoralState.NO_CORAL;

  /* ### Algae Pivot ### */
  @Getter @Setter private AlgaePivotState algaePivotState = AlgaePivotState.INTAKE_FLOOR;

  /* ### Vision ### */
  @Getter @Setter private int limelightImuMode = 0;
  @Getter @Setter private boolean isRedAlliance = true;
}
