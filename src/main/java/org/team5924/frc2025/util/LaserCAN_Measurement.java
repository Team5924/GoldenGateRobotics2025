/*
 * LaserCAN_Measurement.java
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

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/**
 * Wrapper class for {@code LaserCAN.Measurement}. Data is stored in a serializable struct for
 * compatibility with AdvantageKit.
 */
public class LaserCAN_Measurement implements StructSerializable {
  public static class LaserCAN_MeasurementStruct implements Struct<LaserCAN_Measurement> {
    @Override
    public Class<LaserCAN_Measurement> getTypeClass() {
      return LaserCAN_Measurement.class;
    }

    @Override
    public String getTypeName() {
      return "struct:LaserCAN_Measurement";
    }

    @Override
    public int getSize() {
      return (kSizeInt32 * 4) + kSizeBool + LaserCAN_ROI.STRUCT.getSize();
    }

    @Override
    public String getSchema() {
      return "int status;int distance_mm;int ambient;boolean is_long;int budget_ms;LaserCAN_ROI roi";
    }

    @Override
    public LaserCAN_Measurement unpack(ByteBuffer bb) {
      LaserCAN_Measurement measurement = new LaserCAN_Measurement();
      measurement.status = bb.getInt();
      measurement.distance = bb.getFloat();
      measurement.ambient = bb.getInt();
      measurement.isLong = bb.get() != 0;
      measurement.budget = bb.getInt();
      measurement.roi = LaserCAN_ROI.STRUCT.unpack(bb);

      return measurement;
    }

    @Override
    public void pack(ByteBuffer bb, LaserCAN_Measurement value) {
      bb.putInt(value.status);
      bb.putFloat(value.distance);
      bb.putInt(value.ambient);
      bb.put(value.isLong ? (byte) 1 : (byte) 0);
      bb.putInt(value.budget);
      LaserCAN_ROI.STRUCT.pack(bb, value.roi);
    }

    @Override
    public boolean isImmutable() {
      return true;
    }
  }

  public int status;
  public float distance;
  public int ambient;
  public boolean isLong;
  public int budget;
  public LaserCAN_ROI roi;

  public LaserCAN_Measurement(
      int status, float distance, int ambient, boolean isLong, int budget, LaserCAN_ROI roi) {
    this.status = status;
    this.distance = distance;
    this.ambient = ambient;
    this.isLong = isLong;
    this.budget = budget;
    this.roi = roi;
  }

  public LaserCAN_Measurement() {
    this(0, 0, 0, false, 0, new LaserCAN_ROI());
  }

  /**
   * Convert a {@code LaserCAN.Measurement} to a {@code LaserCAN_Measurement} in order to serialize
   * it for use with AdvantageKit.
   *
   * @param measurement Measurement data from LaserCAN sensor.
   * @return Measurement in serializable struct format.
   */
  public static LaserCAN_Measurement fromLaserCAN(Measurement measurement) {
    return new LaserCAN_Measurement(
        measurement.status,
        measurement.distance_mm,
        measurement.ambient,
        measurement.is_long,
        measurement.budget_ms,
        LaserCAN_ROI.fromLaserCAN(measurement.roi));
  }

  public static final LaserCAN_MeasurementStruct STRUCT = new LaserCAN_MeasurementStruct();
}
