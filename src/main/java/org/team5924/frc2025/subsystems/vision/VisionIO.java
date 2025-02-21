/*
 * VisionIO.java
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

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public double botPoseRotationRadians = 0.0;

    public double computedBotPoseX = 0.0;
    public double computedBotPoseY = 0.0;
    public double totalFiducials = 0.0;

    public double frontCameraPoseX = 0.0;
    public double frontCameraPoseY = 0.0;
    public double frontCameraFiducials = 0.0;

    public double backCameraPoseX = 0.0;
    public double backCameraPoseY = 0.0;
    public double backCameraFiducials = 0.0;

    public double lowestTagAmbiguityFront = 1;
    public double lowestTagAmbiguityBack = 1;

    public double avgFrontCameraTagArea = 0.0;
    public double avgBackCameraTagArea = 0.0;

    public Pose2d botPose2d;

    public double aprilTagPipelineLatencySeconds = 0.0;
    public double aprilTagCaptureLatencySeconds = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
