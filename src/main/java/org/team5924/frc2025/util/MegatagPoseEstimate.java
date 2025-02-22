/*
 * MegatagPoseEstimate.java
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

package org.team5924.frc2025.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;

public class MegatagPoseEstimate implements StructSerializable {
  public static class MegatagPoseEstimateStruct implements Struct<MegatagPoseEstimate> {
    public Pose2d fieldToCamera = MathHelpers.kPose2dZero;
    public double timestampSeconds;
    public double latency;
    public int tagCount;
    public double tagSpan;
    public double avgTagDist;
    public double avgTagArea;
    public boolean isFrontLimelight;

    @Override
    public Class<MegatagPoseEstimate> getTypeClass() {
      return MegatagPoseEstimate.class;
    }

    @Override
    public String getTypeName() {
      return "MegatagPoseEstimate";
    }

    @Override
    public String getTypeString() {
      return "struct:MegatagPoseEstimate";
    }

    @Override
    public int getSize() {
      return Pose2d.struct.getSize() + kSizeDouble * 5 + kSizeInt32 + kSizeBool;
    }

    @Override
    public String getSchema() {
      return "Pose2d fieldToCamera;double timestampSeconds;double latency;int tagCount;double tagSpan;double avgTagDist;double avgTagArea;bool isFrontLimelight;";
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] {Pose2d.struct};
    }

    @Override
    public MegatagPoseEstimate unpack(ByteBuffer bb) {
      if (bb.remaining() < getSize()) {
        throw new BufferUnderflowException();
      }
      MegatagPoseEstimate rv = new MegatagPoseEstimate();
      rv.fieldToCamera = Pose2d.struct.unpack(bb);
      rv.timestampSeconds = bb.getDouble();
      rv.latency = bb.getDouble();
      rv.tagCount = bb.getInt();
      rv.tagSpan = bb.getDouble();
      rv.avgTagDist = bb.getDouble();
      rv.avgTagArea = bb.getDouble();
      rv.isFrontLimelight = bb.get() != 0;
      return rv;
    }

    @Override
    public void pack(ByteBuffer bb, MegatagPoseEstimate value) {
      Pose2d.struct.pack(bb, value.fieldToCamera);
      bb.putDouble(value.timestampSeconds);
      bb.putDouble(value.latency);
      bb.putInt(value.tagCount);
      bb.putDouble(value.tagSpan);
      bb.putDouble(value.avgTagDist);
      bb.putDouble(value.avgTagArea);
      bb.put((byte) (value.isFrontLimelight ? 1 : 0));
    }
  }

  public Pose2d fieldToCamera = MathHelpers.kPose2dZero;
  public double timestampSeconds;
  public double latency;
  public int tagCount;
  public double tagSpan;
  public double avgTagDist;
  public double avgTagArea;
  public boolean isFrontLimelight;

  public MegatagPoseEstimate() {}

  public static MegatagPoseEstimate fromLimelight(
      LimelightHelpers.PoseEstimate poseEstimate, boolean isFrontLimelight) {
    if (poseEstimate == null) {
      System.out.println("null megatag!!");
      return null;
    }

    MegatagPoseEstimate rv = new MegatagPoseEstimate();
    rv.fieldToCamera = poseEstimate.pose;
    if (rv.fieldToCamera == null) rv.fieldToCamera = MathHelpers.kPose2dZero;
    rv.timestampSeconds = poseEstimate.timestampSeconds;
    rv.latency = poseEstimate.latency;
    rv.tagCount = poseEstimate.tagCount;
    rv.tagSpan = poseEstimate.tagSpan;
    rv.avgTagDist = poseEstimate.avgTagDist;
    rv.avgTagArea = poseEstimate.avgTagArea;
    rv.isFrontLimelight = isFrontLimelight;

    return rv;
  }

  public static final MegatagPoseEstimateStruct struct = new MegatagPoseEstimateStruct();
}
