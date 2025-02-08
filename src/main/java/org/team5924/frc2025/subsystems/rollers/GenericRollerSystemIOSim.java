/*
 * GenericRollerSystemIOSim.java
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

package org.team5924.frc2025.subsystems.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team5924.frc2025.Constants;

public class GenericRollerSystemIOSim implements GenericRollerSystemIO {
  private final DCMotorSim motorSim;
  private double appliedVoltage = 0.0;
  private static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public GenericRollerSystemIOSim(DCMotor motorModel, double reduction, double moi) {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ROLLER_GEARBOX, Constants.CORAL_IN_AND_OUT_REDUCTION, 0.001),
            ROLLER_GEARBOX);
  }

  @Override
  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    motorSim.update(Constants.LOOP_PERIODIC_SECONDS);
    inputs.positionRads = motorSim.getAngularPositionRad();
    inputs.velocityRadsPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
