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

import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2025.util.FiducialObservation;
import org.team5924.frc2025.util.MegatagPoseEstimate;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean frontLimelightSeesTarget = false;
    public boolean backLimelightSeesTarget = false;

    public FiducialObservation[] frontFiducials = new FiducialObservation[] {};
    public FiducialObservation[] backFiducials = new FiducialObservation[] {};

    public MegatagPoseEstimate megatag2PoseEstimatesFront = new MegatagPoseEstimate();
    public MegatagPoseEstimate megatag2PoseEstimatesBack = new MegatagPoseEstimate();

    public int frontTagCount = 0;
    public int backTagCount = 0;

    public double lowestTagAmbiguityFront = 1;
    public double lowestTagAmbiguityBack = 1;

    public double avgFrontCameraTagArea = 0.0;
    public double avgBackCameraTagArea = 0.0;

    public double FrontAprilTagPipelineLatencySeconds = 0.0;
    public double FrontAprilTagCaptureLatencySeconds = 0.0;

    public double BackAprilTagPipelineLatencySeconds = 0.0;
    public double BackAprilTagCaptureLatencySeconds = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
