/*
 * GenericRollerSystem.java
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

package org.team5924.frc2025.subsystems.rollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public abstract class GenericRollerSystem<G extends GenericRollerSystem.VoltageState>
    extends SubsystemBase {
  public interface VoltageState {
    DoubleSupplier getVoltageSupplier();
  }

  public abstract G getState();

  private G lastState;

  private final String name;

  @Getter private final GenericRollerSystemIO io;
  protected final GenericRollerSystemIOInputsAutoLogged inputs =
      new GenericRollerSystemIOInputsAutoLogged();

  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();

  public GenericRollerSystem(String name, GenericRollerSystemIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.motorConnected);

    if (getState() != lastState) {
      stateTimer.reset();
      lastState = getState();
    }

    io.runVolts(getState().getVoltageSupplier().getAsDouble());
    Logger.recordOutput("Rollers/" + name + "Goal", getState().toString());
  }
}
