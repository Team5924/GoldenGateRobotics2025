/*
 * Drive.java
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

package org.team5924.frc2025.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.Constants.Mode;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.generated.TunerConstantsGamma;
import org.team5924.frc2025.util.Elastic;
import org.team5924.frc2025.util.Elastic.Notification;
import org.team5924.frc2025.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2025.util.LocalADStarAK;
import org.team5924.frc2025.util.VisionFieldPoseEstimate;
import org.team5924.frc2025.util.swerve.SwerveSetpoint;
import org.team5924.frc2025.util.swerve.SwerveSetpointGenerator;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstantsGamma.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(
                  TunerConstantsGamma.FrontLeft.LocationX, TunerConstantsGamma.FrontLeft.LocationY),
              Math.hypot(
                  TunerConstantsGamma.FrontRight.LocationX,
                  TunerConstantsGamma.FrontRight.LocationY)),
          Math.max(
              Math.hypot(
                  TunerConstantsGamma.BackLeft.LocationX, TunerConstantsGamma.BackLeft.LocationY),
              Math.hypot(
                  TunerConstantsGamma.BackRight.LocationX,
                  TunerConstantsGamma.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 58.4;
  private static final double ROBOT_MOI = 4.39;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstantsGamma.FrontLeft.WheelRadius,
              TunerConstantsGamma.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstantsGamma.FrontLeft.DriveMotorGearRatio),
              TunerConstantsGamma.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final Notification gyroDisconnectedNotification =
      new Notification(
          NotificationLevel.ERROR,
          "Gyro Disconnected",
          "Disconnected gyro, using kinematics as fallback.");

  boolean isFlipped =
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d(isFlipped ? 0 : Math.PI);
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  private final Field2d field = new Field2d();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstantsGamma.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstantsGamma.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstantsGamma.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstantsGamma.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5, 0, 0.3)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    setpointGenerator = new SwerveSetpointGenerator(kinematics, getModuleTranslations());
    previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates());

    SmartDashboard.putData("Field", field);

    SmartDashboard.putData(
        "Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                "Front Left Angle", () -> modules[0].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Front Left Velocity", () -> modules[0].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                "Front Right Angle", () -> modules[1].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Front Right Velocity", () -> modules[1].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                "Back Left Angle", () -> modules[2].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Back Left Velocity", () -> modules[2].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                "Back Right Angle", () -> modules[3].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Back Right Velocity", () -> modules[3].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
          }
        });
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    if (!gyroInputs.connected && Constants.currentMode != Mode.SIM)
      Elastic.sendNotification(gyroDisconnectedNotification);

    // Update RobotState
    RobotState.getInstance().setOdometryPose(getPose());

    field.setRobotPose(getPose());

    VisionFieldPoseEstimate visionPoseFrontLeft =
        RobotState.getInstance().getEstimatedPoseFrontLeft();
    VisionFieldPoseEstimate visionPoseFrontRight =
        RobotState.getInstance().getEstimatedPoseFrontRight();
    VisionFieldPoseEstimate visionPoseBack = RobotState.getInstance().getEstimatedPoseBack();

    if (visionPoseFrontLeft != null && visionPoseFrontRight == null && visionPoseBack == null) {
      addVisionMeasurement(
          visionPoseFrontLeft.getVisionRobotPoseMeters(),
          visionPoseFrontLeft.getTimestampSeconds(),
          visionPoseFrontLeft.getVisionMeasurementStdDevs());
      RobotState.getInstance().setEstimatedPoseFrontLeft(null);
    } else if (visionPoseFrontRight != null
        && visionPoseFrontLeft == null
        && visionPoseBack == null) {
      addVisionMeasurement(
          visionPoseFrontRight.getVisionRobotPoseMeters(),
          visionPoseFrontRight.getTimestampSeconds(),
          visionPoseFrontRight.getVisionMeasurementStdDevs());
      RobotState.getInstance().setEstimatedPoseFrontRight(null);
    } else if (visionPoseBack != null
        && visionPoseFrontLeft == null
        && visionPoseFrontRight == null) {
      addVisionMeasurement(
          visionPoseBack.getVisionRobotPoseMeters(),
          visionPoseBack.getTimestampSeconds(),
          visionPoseBack.getVisionMeasurementStdDevs());
      RobotState.getInstance().setEstimatedPoseBack(null);
    } else {
      VisionFieldPoseEstimate latestPose = visionPoseFrontLeft;

      if (visionPoseFrontRight != null
          && (latestPose == null
              || visionPoseFrontRight.getTimestampSeconds() > latestPose.getTimestampSeconds())) {
        latestPose = visionPoseFrontRight;
      }

      if (visionPoseBack != null
          && (latestPose == null
              || visionPoseBack.getTimestampSeconds() > latestPose.getTimestampSeconds())) {
        latestPose = visionPoseBack;
      }

      if (latestPose != null) {
        addVisionMeasurement(
            latestPose.getVisionRobotPoseMeters(),
            latestPose.getTimestampSeconds(),
            latestPose.getVisionMeasurementStdDevs());

        RobotState.getInstance().setEstimatedPoseFrontLeft(null);
        RobotState.getInstance().setEstimatedPoseFrontRight(null);
        RobotState.getInstance().setEstimatedPoseBack(null);
      }
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    previousSetpoint =
        setpointGenerator.generateSetpoint(
            TunerConstantsGamma.moduleLimitsFree,
            previousSetpoint,
            discreteSpeeds,
            Constants.LOOP_PERIODIC_SECONDS);
    SwerveModuleState[] setpointStates = previousSetpoint.moduleStates();
    // SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void runPathVelocity(ChassisSpeeds speeds, DriveFeedforwards ff) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, TunerConstantsGamma.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstantsGamma.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(
          TunerConstantsGamma.FrontLeft.LocationX, TunerConstantsGamma.FrontLeft.LocationY),
      new Translation2d(
          TunerConstantsGamma.FrontRight.LocationX, TunerConstantsGamma.FrontRight.LocationY),
      new Translation2d(
          TunerConstantsGamma.BackLeft.LocationX, TunerConstantsGamma.BackLeft.LocationY),
      new Translation2d(
          TunerConstantsGamma.BackRight.LocationX, TunerConstantsGamma.BackRight.LocationY)
    };
  }
}
