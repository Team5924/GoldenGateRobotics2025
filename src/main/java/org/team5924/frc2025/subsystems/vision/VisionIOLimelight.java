/*
 * VisionIOLimelight.java
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

package org.team5924.frc2025.subsystems.vision;

import org.team5924.frc2025.Constants;
import org.team5924.frc2025.util.LimelightHelpers;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {
  public VisionIOLimelight() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    LimelightHelpers.PoseEstimate frontCameraEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("front");
    LimelightHelpers.PoseEstimate backCameraEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("back");

    double lowestTagAmbiguityFront = 1;
    double lowestTagAmbiguityBack = 1;

    if (frontCameraEstimate != null) {
      inputs.frontCameraPoseX = frontCameraEstimate.pose.getX();
      inputs.frontCameraPoseY = frontCameraEstimate.pose.getY();
      inputs.frontCameraFiducials = frontCameraEstimate.tagCount;

      for (LimelightHelpers.RawFiducial rawFiducial : frontCameraEstimate.rawFiducials) {
        double frontTagAmbiguity = rawFiducial.ambiguity;
        if (frontTagAmbiguity < lowestTagAmbiguityFront) {
          lowestTagAmbiguityFront = frontTagAmbiguity;
        }
      }

      inputs.avgFrontCameraTagArea = frontCameraEstimate.avgTagArea;
      inputs.lowestTagAmbiguityFront = lowestTagAmbiguityFront;
      inputs.botPoseRotationRadians = frontCameraEstimate.pose.getRotation().getRadians();
    }

    if (backCameraEstimate != null) {
      inputs.backCameraPoseX = backCameraEstimate.pose.getX();
      inputs.backCameraPoseY = backCameraEstimate.pose.getY();
      inputs.backCameraFiducials = backCameraEstimate.tagCount;

      for (LimelightHelpers.RawFiducial rawFiducial : backCameraEstimate.rawFiducials) {
        double backTagAmbiguity = rawFiducial.ambiguity;
        if (backTagAmbiguity < lowestTagAmbiguityBack) {
          lowestTagAmbiguityBack = backTagAmbiguity;
        }
      }

      inputs.avgBackCameraTagArea = backCameraEstimate.avgTagArea;
      inputs.lowestTagAmbiguityBack = lowestTagAmbiguityBack;
    }

    // inputs.totalFiducials = inputs.frontCameraFiducials + inputs.backCameraFiducials;

    if ((frontCameraEstimate != null && backCameraEstimate != null)
        && frontCameraEstimate.tagCount > backCameraEstimate.tagCount) {
      inputs.computedBotPoseX = frontCameraEstimate.pose.getX();
      inputs.computedBotPoseY = frontCameraEstimate.pose.getY();
    } else if ((frontCameraEstimate != null && backCameraEstimate != null)
        && backCameraEstimate.tagCount > frontCameraEstimate.tagCount) {
      inputs.computedBotPoseX = backCameraEstimate.pose.getX();
      inputs.computedBotPoseY = backCameraEstimate.pose.getY();
    } else if (frontCameraEstimate != null && backCameraEstimate != null) {
      inputs.computedBotPoseX =
          (frontCameraEstimate.pose.getX() + backCameraEstimate.pose.getX()) / 2;
      inputs.computedBotPoseY =
          (frontCameraEstimate.pose.getY() + backCameraEstimate.pose.getY()) / 2;
    }

    inputs.aprilTagPipelineLatencySeconds =
        LimelightHelpers.getLatency_Pipeline(Constants.APRIL_TAG_LIMELIGHT_NAME_FRONT) / 1000;
    inputs.aprilTagCaptureLatencySeconds =
        LimelightHelpers.getLatency_Capture(Constants.APRIL_TAG_LIMELIGHT_NAME_FRONT) / 1000;
  }
}
