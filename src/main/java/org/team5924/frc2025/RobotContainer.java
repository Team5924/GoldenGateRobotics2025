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

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Set;
import org.team5924.frc2025.commands.coralInAndOut.TeleopShoot;
import org.team5924.frc2025.commands.drive.DriveCommands;
import org.team5924.frc2025.commands.elevator.RunElevator;
import org.team5924.frc2025.generated.TunerConstantsGamma;
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
import org.team5924.frc2025.subsystems.elevator.ElevatorIOTalonFXGamma;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOut;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOutIO;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOutIOKrakenFOC;
import org.team5924.frc2025.subsystems.rollers.CoralInAndOut.CoralInAndOutIOSim;
import org.team5924.frc2025.subsystems.vision.Vision;
import org.team5924.frc2025.subsystems.vision.VisionIO;
import org.team5924.frc2025.subsystems.vision.VisionIOLimelight;

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
  private final Vision vision;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstantsGamma.FrontLeft),
                new ModuleIOTalonFX(TunerConstantsGamma.FrontRight),
                new ModuleIOTalonFX(TunerConstantsGamma.BackLeft),
                new ModuleIOTalonFX(TunerConstantsGamma.BackRight));
        climber = new Climber(new ClimberIOTalonFX());
        coralInAndOut = new CoralInAndOut(new CoralInAndOutIOKrakenFOC());
        elevator = new Elevator(new ElevatorIOTalonFXGamma() {});
        vision = new Vision(new VisionIOLimelight());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstantsGamma.FrontLeft),
                new ModuleIOSim(TunerConstantsGamma.FrontRight),
                new ModuleIOSim(TunerConstantsGamma.BackLeft),
                new ModuleIOSim(TunerConstantsGamma.BackRight));
        climber = new Climber(new ClimberIOSim());
        coralInAndOut = new CoralInAndOut(new CoralInAndOutIOSim());
        elevator = new Elevator(new ElevatorIO() {});
        vision = new Vision(new VisionIO() {});
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
        vision = new Vision(new VisionIO() {});
        break;
    }

    NamedCommands.registerCommand(
        "Run Shooter",
        Commands.runOnce(() -> coralInAndOut.setGoalState(CoralInAndOut.CoralState.SHOOTING_L4)));
    NamedCommands.registerCommand(
        "Stop CoralInAndOut",
        Commands.runOnce(() -> coralInAndOut.setGoalState(CoralInAndOut.CoralState.NO_CORAL)));
    NamedCommands.registerCommand(
        "Run Intake",
        Commands.runOnce(() -> coralInAndOut.setGoalState(CoralInAndOut.CoralState.INTAKING)));
    NamedCommands.registerCommand(
        "Elevator Height L4",
        Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.L4)));
    NamedCommands.registerCommand(
        "Elevator Height L3",
        Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.L3)));
    NamedCommands.registerCommand(
        "Elevator Height Intake",
        Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.INTAKE)));

    new EventTrigger("Elevator Height L4 Trigger")
        .onTrue(Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.L4)));

    new EventTrigger("Elevator Height Intake Trigger")
        .onTrue(Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.INTAKE)));

    // Set up auto routines
    boolean isCompetition = true;

    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer
    autoChooser =
        AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) ->
                isCompetition ? stream.filter(auto -> auto.getName().startsWith("2")) : stream);

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
    autoChooser.addOption(
        "Elevator SysId (Quasistatic Forward)",
        elevator.upSysId.quasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Elevator SysId (Quasistatic Reverse)",
        elevator.downSysId.quasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Elevator SysId (Dynamic Forward)",
        elevator.upSysId.dynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Elevator SysId (Dynamic Reverse)",
        elevator.downSysId.dynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY() * .8,
            () -> -driveController.getLeftX() * .8,
            () -> -driveController.getRightX() * .8));

    // Nope. It's slow mode now. Quarter speed
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY() * .25,
                () -> -driveController.getLeftX() * .25,
                () -> -driveController.getRightX() * .25));

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

    driveController
        .leftBumper()
        .whileTrue(
            new DeferredCommand(() -> DriveCommands.driveToReef(drive, true), Set.of(drive)));

    driveController
        .rightBumper()
        .whileTrue(
            new DeferredCommand(() -> DriveCommands.driveToReef(drive, false), Set.of(drive)));

    driveController
        .rightTrigger()
        .whileTrue(
            DriveCommands.turnToRightCoralStation(
                drive, () -> -driveController.getLeftY(), () -> -driveController.getLeftX()));

    driveController
        .leftTrigger()
        .whileTrue(
            DriveCommands.turnToLeftCoralStation(
                drive, () -> -driveController.getLeftY(), () -> -driveController.getLeftX()));
    // Coral In and Out

    driveController.y().onTrue(new TeleopShoot(coralInAndOut).withTimeout(Seconds.of(1)));
    driveController
        .leftTrigger()
        .onFalse(
            Commands.runOnce(() -> coralInAndOut.setGoalState(CoralInAndOut.CoralState.NO_CORAL)));

    operatorController
        .rightTrigger()
        .onTrue(
            Commands.runOnce(() -> coralInAndOut.setGoalState(CoralInAndOut.CoralState.INTAKING)));
    operatorController
        .rightTrigger()
        .onFalse(
            Commands.runOnce(
                () -> coralInAndOut.setGoalState(CoralInAndOut.CoralState.STORED_CORAL_IN_INTAKE)));

    // Elevator
    elevator.setDefaultCommand(new RunElevator(elevator, operatorController::getLeftY));
    operatorController
        .a()
        .onTrue(Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.STOW)));
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
    operatorController
        .rightBumper()
        .onTrue(Commands.runOnce(() -> elevator.setGoalState(Elevator.ElevatorState.INTAKE)));

    // Vision
    // vision.setDefaultCommand(new RunVisionPoseEstimation(drive, vision).ignoringDisable(true));

    // Climber
    // Dpad Down
    driveController
        .pov(180)
        .onTrue(Commands.runOnce(() -> climber.setGoalState(Climber.ClimberState.CLIMB)));

    // Dpad Up
    driveController
        .pov(0)
        .onTrue(Commands.runOnce(() -> climber.setGoalState(Climber.ClimberState.REVERSE_CLIMB)));

    // No Dpad Up or Dpad Down
    driveController
        .pov(180)
        .or(driveController.pov(0))
        .onFalse(Commands.runOnce(() -> climber.handleNoInputState()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
