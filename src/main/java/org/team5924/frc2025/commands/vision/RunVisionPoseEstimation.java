/*
 * RunVisionPoseEstimation.java
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

package org.team5924.frc2025.commands.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.subsystems.drive.Drive;
import org.team5924.frc2025.subsystems.vision.Vision;
import org.team5924.frc2025.util.MegatagPoseEstimate;

public class RunVisionPoseEstimation extends Command {
  private final Drive drive;
  private final Vision vision;

  /** Creates a new RunVisionPoseEstimation. */
  public RunVisionPoseEstimation(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    MegatagPoseEstimate estimatedPose = vision.getBotPose2dBlue();
    if (estimatedPose == null) {
      Logger.recordOutput("Vision Error", "Failed to get pose estimate");
      return;
    }
    Logger.recordOutput("Vision Pose", estimatedPose.fieldToCamera);
    if (isPoseValid(estimatedPose)
        && isVisionReliable(estimatedPose)
        && estimatedPose.avgTagDist < 1) {
      if (DriverStation.isDisabled()) {
        drive.setPose(estimatedPose.fieldToCamera);
      } else {
        drive.addVisionMeasurement(
            estimatedPose.fieldToCamera,
            Timer.getFPGATimestamp()
                - (estimatedPose.isFrontLimelight
                    ? vision.getLatencySecondsFront()
                    : vision.getLatencySecondsBack()));
      }
    }
  }

  private boolean isPoseValid(MegatagPoseEstimate pose) {
    return pose.fieldToCamera.getX() != 0 && pose.fieldToCamera.getY() != 0;
  }

  private boolean isVisionReliable(MegatagPoseEstimate pose) {
    int fiducialsSpotted =
        pose.isFrontLimelight
            ? vision.getNumberFiducialsSpottedFront()
            : vision.getNumberFiducialsSpottedBack();
    double lowestAmbiguity =
        pose.isFrontLimelight
            ? vision.getLowestTagAmbiguityFront()
            : vision.getLowestTagAmbiguityBack();

    return (fiducialsSpotted == 1 && lowestAmbiguity < 0.1)
        || (fiducialsSpotted >= 2 && lowestAmbiguity < 0.3);
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
