/*
 * LaserCAN_ROI.java
 */

/* 
 * Copyright (C) 2024-2025 Team 5924 - Golden Gate Robotics and/or its affiliates.
 *
 * This file, and the associated project, are offered under the GNU General
 * Public License v3.0. A copy of this license can be found in LICENSE.md
 * at the root of this project.
 *
 * If this file has been seperated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025.util;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class LaserCAN_ROI implements StructSerializable {
  public static class LaserCAN_ROIStruct implements Struct<LaserCAN_ROI> {
    @Override
    public Class<LaserCAN_ROI> getTypeClass() {
      return LaserCAN_ROI.class;
    }

    @Override
    public String getTypeName() {
      return "struct:LaserCAN_ROI";
    }

    @Override
    public int getSize() {
      return kSizeInt32 * 4;
    }

    @Override
    public String getSchema() {
      return "int x;int y;int w;int h;";
    }

    @Override
    public LaserCAN_ROI unpack(ByteBuffer bb) {
      LaserCAN_ROI roi = new LaserCAN_ROI();
      roi.x = bb.getInt();
      roi.y = bb.getInt();
      roi.w = bb.getInt();
      roi.h = bb.getInt();
      return roi;
    }

    @Override
    public void pack(ByteBuffer bb, LaserCAN_ROI value) {
      bb.putInt(value.x);
      bb.putInt(value.y);
      bb.putInt(value.w);
      bb.putInt(value.h);
    }

    @Override
    public boolean isImmutable() {
      return true;
    }
  }

  public int x;
  public int y;
  public int w;
  public int h;

  public LaserCAN_ROI(int x, int y, int w, int h) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
  }

  public LaserCAN_ROI() {
    this(0, 0, 0, 0);
  }

  public static LaserCAN_ROI fromLaserCAN(RegionOfInterest roi) {
    return new LaserCAN_ROI(roi.x, roi.y, roi.w, roi.h);
  }

  public static final LaserCAN_ROIStruct STRUCT = new LaserCAN_ROIStruct();
}
