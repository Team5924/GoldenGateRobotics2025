/*
 * RobotContainer.java
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

package org.team5924.frc2025;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team5924.frc2025.commands.DriveCommands;
import org.team5924.frc2025.commands.elevator.RunElevator;
import org.team5924.frc2025.generated.TunerConstants;
import org.team5924.frc2025.subsystems.climber.Climber;
import org.team5924.frc2025.subsystems.climber.ClimberIO;
import org.team5924.frc2025.subsystems.climber.ClimberIOSim;
import org.team5924.frc2025.subsystems.climber.ClimberIOTalonFX;
import org.team5924.frc2025.subsystems.drive.Drive;
import org.team5924.frc2025.subsystems.drive.GyroIO;
import org.team5924.frc2025.subsystems.drive.GyroIOPigeon2;
import org.team5924.frc2025.subsystems.drive.ModuleIO;
import org.team5924.frc2025.subsystems.drive.ModuleIOSim;
import org.team5924.frc2025.subsystems.drive.ModuleIOTalonFX;
import org.team5924.frc2025.subsystems.elevator.Elevator;
import org.team5924.frc2025.subsystems.elevator.ElevatorIO;
import org.team5924.frc2025.subsystems.elevator.ElevatorIOTalonFX;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOut;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOutIO;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOutIOKrakenFOC;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOutIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Climber climber;
  private final CoralInAndOut coralInAndOut;
  private final Elevator elevator;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * Initializes the RobotContainer, setting up subsystems, input devices, and autonomous routines.
   * 
   * <p>This constructor instantiates the drive, climber, coral in/out, and elevator subsystems using mode-specific IO
   * implementations:
   * <ul>
   *   <li>REAL mode: Hardware-specific IO implementations for actual robot operation.</li>
   *   <li>SIM mode: Simulation IO implementations to support physics-based simulation.</li>
   *   <li>Default (REPLAYED) mode: Disabled or default IO implementations for replayed operation.</li>
   * </ul>
   * Additionally, it configures the autonomous command chooser with various drive characterization and system
   * identification routines, and binds operator controls by invoking {@link #configureButtonBindings()}.
   * </p>
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        climber = new Climber(new ClimberIOTalonFX());
        coralInAndOut = new CoralInAndOut(new CoralInAndOutIOKrakenFOC());
        elevator = new Elevator(new ElevatorIOTalonFX() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        climber = new Climber(new ClimberIOSim());
        coralInAndOut = new CoralInAndOut(new CoralInAndOutIOSim());
        elevator = new Elevator(new ElevatorIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        climber = new Climber(new ClimberIO() {});
        coralInAndOut = new CoralInAndOut(new CoralInAndOutIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Assigns controller buttons to commands across multiple subsystems including drive, coral control, elevator,
   * and climber.
   *
   * <p>Configured bindings:
   * <ul>
   *   <li><b>Drive Subsystem:</b> Sets the default joystick drive command; holds A button for angle lock (0°);
   *       X button to stop drive; and B button to reset the gyro by reinitializing the drive pose.</li>
   *   <li><b>Coral Control:</b> Uses the operator's triggers to set the coral state's shooting when the left trigger
   *       is pressed and intaking when the right trigger is pressed. When released, resets the state to NO_CORAL.</li>
   *   <li><b>Elevator Subsystem:</b> Establishes the default command for joystick-based elevator control and maps the
   *       A, B, X, Y, and left bumper buttons to specific elevator goal states (L1, L2, L3, L4, and MANUAL, respectively).</li>
   *   <li><b>Climber Subsystem:</b> Maps the drive controller's D-pad: pressing down (pov 180°) initiates CLIMB,
   *       pressing up (pov 0°) initiates REVERSE_CLIMB, while releasing both reverts the climber to STOW if already
   *       in STOW or sets it to READY_TO_CLIMB otherwise.</li>
   * </ul>
   *
   * <p>The method configures these mappings using lambda expressions and the command framework to ensure responsive,
   * context-sensitive control of each subsystem.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    // Lock to 0° when A button is held
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Coral In and Out
    operatorController
        .leftTrigger()
        .onTrue(
            Commands.runOnce(() -> coralInAndOut.setGoalState(CoralInAndOut.CoralState.SHOOTING)));
    operatorController
        .rightTrigger()
        .onTrue(
            Commands.runOnce(() -> coralInAndOut.setGoalState(CoralInAndOut.CoralState.INTAKING)));
    operatorController
        .leftTrigger()
        .onFalse(
            Commands.runOnce(() -> coralInAndOut.setGoalState(CoralInAndOut.CoralState.NO_CORAL)));
    operatorController
        .rightTrigger()
        .onFalse(
            Commands.runOnce(() -> coralInAndOut.setGoalState(CoralInAndOut.CoralState.NO_CORAL)));

    // Elevator
    elevator.setDefaultCommand(new RunElevator(elevator, operatorController::getLeftY));
    operatorController
        .a()
        .onTrue(Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.L1)));
    operatorController
        .b()
        .onTrue(Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.L2)));
    operatorController
        .x()
        .onTrue(Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.L3)));
    operatorController
        .y()
        .onTrue(Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.L4)));
    operatorController
        .leftBumper()
        .onTrue(Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.MANUAL)));

    // Climber
    // Dpad Down
    driveController
        .pov(180)
        .onTrue(Commands.runOnce(() -> climber.setGoalState(Climber.ClimberState.CLIMB)));
    // .finallyDo(() -> climber.setVoltageMultiplier(-1)));

    // Dpad Up
    driveController
        .pov(0)
        .onTrue(Commands.runOnce(() -> climber.setGoalState(Climber.ClimberState.REVERSE_CLIMB)));
    // .finallyDo(() -> climber.setVoltageMultiplier(1)));

    // No Dpad Up or Dpad Down
    driveController
        .pov(180)
        .or(driveController.pov(0))
        .onFalse(
            Commands.runOnce(
                () ->
                    climber.setGoalState(
                        climber.getGoalState() == Climber.ClimberState.STOW
                            ? Climber.ClimberState.STOW
                            : Climber.ClimberState.READY_TO_CLIMB)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
