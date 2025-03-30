package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Swerve.SimConstants.Mode;
import frc.robot.subsystems.Swerve.Controller.HolonomicDriveWithPIDController;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstantsNew;
import frc.robot.util.ControllerUtil;
import frc.robot.util.LocalADStarAK;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private static final double ROBOT_MASS_KG = 56.7;
  private static final double ROBOT_MOI = 3.00241;
  private static final double WHEEL_COF = 1.0;
  private static final RobotConfig PP_CONFIG = new RobotConfig(
      ROBOT_MASS_KG,
      ROBOT_MOI,
      new ModuleConfig(
          TunerConstantsNew.FrontLeft.WheelRadius,
          TunerConstantsNew.kSpeedAt12Volts.in(MetersPerSecond),
          WHEEL_COF,
          DCMotor.getKrakenX60Foc(1)
              .withReduction(TunerConstantsNew.FrontLeft.DriveMotorGearRatio),
          TunerConstantsNew.FrontLeft.SlipCurrent,
          1),
      getModuleTranslations());
  // TunerConstantsNew doesn't include these constants, so they are declared
  // locally
  static final double ODOMETRY_FREQUENCY = new CANBus(TunerConstantsNew.DrivetrainConstants.CANBusName).isNetworkFD()
      ? 250.0
      : 100.0;
  public static final double DRIVE_BASE_RADIUS = Math.max(
      Math.max(
          Math.hypot(TunerConstantsNew.FrontLeft.LocationX, TunerConstantsNew.FrontLeft.LocationY),
          Math.hypot(TunerConstantsNew.FrontRight.LocationX, TunerConstantsNew.FrontRight.LocationY)),
      Math.max(
          Math.hypot(TunerConstantsNew.BackLeft.LocationX, TunerConstantsNew.BackLeft.LocationY),
          Math.hypot(TunerConstantsNew.BackRight.LocationX, TunerConstantsNew.BackRight.LocationY)));

  private final Field2d m_field = new Field2d();
  private final PIDController headingController = new PIDController(4, 0.0, 0.0);

  private boolean holonomicControllerActive = false;
  private Pose2d holonomicPoseTarget = new Pose2d();
  private final HolonomicDriveWithPIDController holonomicDriveWithPIDController;
  private final HolonomicDriveWithPIDController holonomicDriveWithPIDControllerBarge;
  private final SysIdRoutine sysId;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
      AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
      lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstantsNew.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstantsNew.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstantsNew.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstantsNew.BackRight);

    this.holonomicDriveWithPIDController = new HolonomicDriveWithPIDController(
        new PIDController(4, 0, 0),
        new PIDController(4, 0, 0),
        headingController,
        0.5,
        new Pose2d(0.03, 0.03, Rotation2d.fromDegrees(1)),
        1);
    
    this.holonomicDriveWithPIDControllerBarge = new HolonomicDriveWithPIDController(
        new PIDController(4, 0, 0),
        new PIDController(4, 0, 0),
        headingController,
        0.35,
        new Pose2d(0.03, 0.03, Rotation2d.fromDegrees(1)),
        0.5);

    SmartDashboard.putData("Field", m_field);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(4, 0.0, 0.0), new PIDConstants(4, 0.0, 0.0)),
        PP_CONFIG,
        () -> {
          var alliance =  DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
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

    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // if (AllianceFlipUtil.shouldFlip()) {
    //   m_field.setRobotPose(AllianceFlipUtil.apply(getPose()));
    // } else {
    //   m_field.setRobotPose(getPose());
    // }
    SmartDashboard.putNumber("Closest Reef", getClosestReefFace());

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
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
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
    gyroDisconnectedAlert.set(!gyroInputs.connected && SimConstants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstantsNew.kSpeedAt12Volts);

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
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  public Command ChoreoAuto(String name) {
      try {
              PathPlannerPath originalPath = PathPlannerPath.fromChoreoTrajectory(name);
              PathPlannerPath finalPath;
              
              if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                  finalPath = originalPath.flipPath();
              }
              else {
                  finalPath = originalPath;
              }
          
          
              return AutoBuilder.followPath(finalPath).alongWith(Commands.runOnce(() -> this.setPose(finalPath.getStartingHolonomicPose().get())));
      } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
      } 
    }
  

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Returns the module positions (turn angles and drive positions) for all of the
   * modules.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
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

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
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

  public void resetPose() {
    poseEstimator.resetPose(new Pose2d(0, 0, this.rawGyroRotation));
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
    return TunerConstantsNew.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(TunerConstantsNew.FrontLeft.LocationX, TunerConstantsNew.FrontLeft.LocationY),
        new Translation2d(TunerConstantsNew.FrontRight.LocationX, TunerConstantsNew.FrontRight.LocationY),
        new Translation2d(TunerConstantsNew.BackLeft.LocationX, TunerConstantsNew.BackLeft.LocationY),
        new Translation2d(TunerConstantsNew.BackRight.LocationX, TunerConstantsNew.BackRight.LocationY)
    };
  }


  public Command driveToPose(Pose2d pose) {
    return Commands.sequence(
        runOnce(() -> {
          holonomicControllerActive = true;
          holonomicDriveWithPIDController.reset(getPose(), getRobotRelativeSpeeds());
        }),
        run(() -> {
          this.holonomicPoseTarget = pose;
          runVelocity(holonomicDriveWithPIDController.calculate(getPose(), holonomicPoseTarget));
        }).until(holonomicDriveWithPIDController::atReference),
        runOnce(this::stop)).finallyDo(() -> holonomicControllerActive = false);
  }

  public Command driveToPoseBarge(Pose2d pose) {
    return Commands.sequence(
        runOnce(() -> {
          holonomicControllerActive = true;
          holonomicDriveWithPIDControllerBarge.reset(getPose(), getRobotRelativeSpeeds());
        }),
        run(() -> {
          this.holonomicPoseTarget = pose;
          runVelocity(holonomicDriveWithPIDControllerBarge.calculate(getPose(), holonomicPoseTarget));
        }).until(holonomicDriveWithPIDControllerBarge::atReference),
        runOnce(this::stop)).finallyDo(() -> holonomicControllerActive = false);
  }

  public Command driveFromCurrentPose(double xValue) {
    return Commands.defer(
      () -> {
        return driveToPoseBarge(new Pose2d(xValue, getPose().getY(), new Rotation2d(0)));
      }, 
      Set.of(this)
      );
  }

  public Command followPPPathCommand(String pathName) {
    return Commands.defer(
      () -> {
        try {
          // Load the path you want to follow using its name in the GUI
          PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

          Pose2d startingPose = path.getStartingHolonomicPose().get();
          
          Logger.recordOutput("Inital Pose", startingPose);
          
          // Create a path following command using AutoBuilder. This will also trigger
          // event markers.
          return AutoBuilder.resetOdom(startingPose).andThen(AutoBuilder.followPath(path));
        } catch (Exception e) {
          DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
          return Commands.none();
        }
      },
      Set.of(this)
    );
  }

  public Command driveRight() {
    return Commands.runOnce(() -> {
      Pose2d currentPose = getPose();
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(0, -0.1)), new Rotation2d());
      driveToPose(endPos);
    });
  }

  public Command driveLeft() {
    return Commands.runOnce(() -> {
      Pose2d currentPose = getPose();
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(0, 0.1)), new Rotation2d());
      driveToPose(endPos);
    });
  }

  public double getDistance(Pose2d target) {
    Pose2d currentPose = getPose();
    double distance = Math
        .sqrt(Math.pow(currentPose.getX() - target.getX(), 2) + Math.pow(currentPose.getY() - target.getY(), 2));
    return distance;
  }

  public Pose2d getClosestReef() {
    Pose2d closestReef = FieldConstants.Reef.reef0;
    final List<Pose2d> reefCenterPosesList = Robot.IsRedAlliance.getAsBoolean()
        ? Arrays.asList(FieldConstants.Reef.redReefs)
        : Arrays.asList(FieldConstants.Reef.blueReefs);
    for (Pose2d reef : reefCenterPosesList) {
      if (getDistance(reef) < getDistance(closestReef)) {
        closestReef = reef;
      }
    }
    return closestReef;
  }

  public int getClosestReefFace() {
    Pose2d closestReef = FieldConstants.Reef.reef0;
    int indexOf = 0;
    final List<Pose2d> reefCenterPosesList = Robot.IsRedAlliance.getAsBoolean()
        ? Arrays.asList(FieldConstants.Reef.redReefs)
        : Arrays.asList(FieldConstants.Reef.blueReefs);
    for (int i = 0; i < reefCenterPosesList.size(); i++) {
      if (getDistance(reefCenterPosesList.get(i)) < getDistance(closestReef)) {
        closestReef = reefCenterPosesList.get(i);
        indexOf = i;
      }
    }
    return indexOf;
  }

  public Command driveToClosestReefScoringFace() {
    final List<Pose2d> reefCenterPosesList = Robot.IsRedAlliance.getAsBoolean()
        ? Arrays.asList(FieldConstants.Reef.redReefs)
        : Arrays.asList(FieldConstants.Reef.blueReefs);

    return Commands.defer(
        () -> {
          final Pose2d currentPose = getPose();
          final Pose2d nearestCoralSide = currentPose.nearest(reefCenterPosesList);

          return driveToPose(nearestCoralSide);
        },
        Set.of(this));

  }

  public Command driveToClosestReefScoringFaceWithTranslate(Transform2d transform2d, CommandPS5Controller controller) {
    final List<Pose2d> reefCenterPosesList = Robot.IsRedAlliance.getAsBoolean() ? Arrays.asList(FieldConstants.Reef.redReefs)
    : Arrays.asList(FieldConstants.Reef.blueReefs);
    return Commands.defer(
        () -> {

          final Pose2d currentPose = getPose();
          final Pose2d nearestCoralSide = currentPose.nearest(reefCenterPosesList);
          Pose2d drivePose = nearestCoralSide.plus(transform2d);

          return driveToPose(drivePose);
        },
        Set.of(this)).finallyDo(() -> ControllerUtil.rumbleForDurationCommand(controller.getHID(), RumbleType.kBothRumble, 0.5, 1.0));

  }

  public Command driveToClosestReefScoringFaceWithTranslate(Transform2d transform2d) {
    final List<Pose2d> reefCenterPosesList = Robot.IsRedAlliance.getAsBoolean() ? Arrays.asList(FieldConstants.Reef.redReefs)
    : Arrays.asList(FieldConstants.Reef.blueReefs);
    return Commands.defer(
        () -> {

          final Pose2d currentPose = getPose();
          final Pose2d nearestCoralSide = currentPose.nearest(reefCenterPosesList);
          Pose2d drivePose = nearestCoralSide.plus(transform2d);

          return driveToPose(drivePose);
        },
        Set.of(this));

  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState());
  }

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(
        getGyroRotation(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        },
        pose);
  }

  public Rotation2d getGyroRotation() {
    return gyroInputs.yawPosition;
  }

}
