package org.team5924.frc2025.subsystems.rollers.shooter;

import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIOSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterIOSim extends GenericRollerSystemIOSim {
  private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
  private static final double reduction = (18.0 / 12.0);
  private static final double moi = 0.001;

  public ShooterIOSim() {
    super(motorModel, reduction, moi);
  }
}