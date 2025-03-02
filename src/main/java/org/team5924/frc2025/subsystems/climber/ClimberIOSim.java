/*
 * ClimberIOSim.java
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

package org.team5924.frc2025.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team5924.frc2025.Constants;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim motorSim;
  private double appliedVoltage = 0.0;
  private static final DCMotor CLIMBER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public ClimberIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                CLIMBER_GEARBOX, Constants.CLIMBER_REDUCTION, Constants.CLIMBER_SIM_MOI),
            CLIMBER_GEARBOX);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    motorSim.update(Constants.LOOP_PERIODIC_SECONDS);
    inputs.leftPositionRads = motorSim.getAngularPositionRad();
    inputs.leftVelocityRadsPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVoltage = appliedVoltage;
    inputs.leftSupplyCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.rightPositionRads = motorSim.getAngularPositionRad();
    inputs.rightVelocityRadsPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.rightAppliedVoltage = appliedVoltage;
    inputs.rightSupplyCurrentAmps = motorSim.getCurrentDrawAmps();
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
