/*
 * Vision.java
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

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.util.MegatagPoseEstimate;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final VisionIO io;

  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  private final BooleanSubscriber allianceSubscriber =
      NetworkTableInstance.getDefault()
          .getTable("FMSInfo")
          .getBooleanTopic("IsRedAlliance")
          .subscribe(true);
  private boolean previousAllianceSubscriberValue = true;

  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    boolean isRedAlliance = allianceSubscriber.get();
    if (isRedAlliance != previousAllianceSubscriberValue) {
      previousAllianceSubscriberValue = isRedAlliance;
      RobotState.getInstance().setRedAlliance(isRedAlliance);
    }
  }

  public MegatagPoseEstimate getBotPose2dBlue() {
    if (inputs.megatag2PoseEstimateFront == null && inputs.megatag2PoseEstimateBack == null) {
      return null;
    }
    if (inputs.megatag2PoseEstimateFront == null) {
      return inputs.megatag2PoseEstimateBack;
    }
    if (inputs.megatag2PoseEstimateBack == null) {
      return inputs.megatag2PoseEstimateFront;
    }
    if (inputs.lowestTagAmbiguityFront < inputs.lowestTagAmbiguityBack) {
      return inputs.megatag2PoseEstimateFront;
    } else {
      return inputs.megatag2PoseEstimateBack;
    }
  }

  public double getLatencySecondsFront() {
    return inputs.frontAprilTagCaptureLatencySeconds + inputs.frontAprilTagPipelineLatencySeconds;
  }

  public double getLatencySecondsBack() {
    return inputs.backAprilTagCaptureLatencySeconds + inputs.backAprilTagPipelineLatencySeconds;
  }

  public double getLowestTagAmbiguityFront() {
    return inputs.lowestTagAmbiguityFront;
  }

  public double getLowestTagAmbiguityBack() {
    return inputs.lowestTagAmbiguityBack;
  }

  public int getNumberFiducialsSpottedFront() {
    return inputs.frontFiducials.length;
  }

  public int getNumberFiducialsSpottedBack() {
    return inputs.backFiducials.length;
  }
}
