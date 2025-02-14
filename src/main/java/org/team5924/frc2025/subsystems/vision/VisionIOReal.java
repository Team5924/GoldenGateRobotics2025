/*
 * VisionIOReal.java
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

import org.team5924.frc2025.util.LimelightHelpers;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {
  public VisionIOReal() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    LimelightHelpers.PoseEstimate frontCameraEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("frontCamera");
    LimelightHelpers.PoseEstimate backCameraEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("backCamera");

    inputs.frontCameraPoseX = frontCameraEstimate.pose.getX();
    inputs.frontCameraPoseY = frontCameraEstimate.pose.getY();

    inputs.backCameraPoseX = backCameraEstimate.pose.getX();
    inputs.backCameraPoseY = backCameraEstimate.pose.getY();

    inputs.frontCameraFiducials = frontCameraEstimate.tagCount;
    inputs.backCameraFiducials = backCameraEstimate.tagCount;
    inputs.totalFiducials = inputs.frontCameraFiducials + inputs.backCameraFiducials;

    double lowestTagAmbiguityFront = 1;
    for (LimelightHelpers.RawFiducial rawFiducial : frontCameraEstimate.rawFiducials) {
      double frontTagAmbiguity = rawFiducial.ambiguity;
      if (frontTagAmbiguity < lowestTagAmbiguityFront) {
        lowestTagAmbiguityFront = frontTagAmbiguity;
      }
    }

    double lowestTagAmbiguityBack = 1;
    for (LimelightHelpers.RawFiducial rawFiducial : backCameraEstimate.rawFiducials) {
      double backTagAmbiguity = rawFiducial.ambiguity;
      if (backTagAmbiguity < lowestTagAmbiguityBack) {
        lowestTagAmbiguityBack = backTagAmbiguity;
      }
    }

    inputs.avgFrontCameraTagArea = frontCameraEstimate.avgTagArea;
    inputs.avgFrontCameraTagArea = frontCameraEstimate.avgTagArea;

    inputs.lowestTagAmbiguityFront = lowestTagAmbiguityFront;
    inputs.lowestTagAmbiguityBack = lowestTagAmbiguityBack;

    if (frontCameraEstimate.tagCount > backCameraEstimate.tagCount) {
      inputs.computedBotPoseX = frontCameraEstimate.pose.getX();
      inputs.computedBotPoseY = frontCameraEstimate.pose.getY();
    } else if (backCameraEstimate.tagCount > frontCameraEstimate.tagCount) {
      inputs.computedBotPoseX = backCameraEstimate.pose.getX();
      inputs.computedBotPoseY = backCameraEstimate.pose.getY();
    } else {
      inputs.computedBotPoseX =
          (frontCameraEstimate.pose.getX() + backCameraEstimate.pose.getX()) / 2;
      inputs.computedBotPoseY =
          (frontCameraEstimate.pose.getY() + backCameraEstimate.pose.getY()) / 2;
    }
    
    inputs.botPoseRotationRadians = frontCameraEstimate.pose.getRotation().getRadians();
  }
}
