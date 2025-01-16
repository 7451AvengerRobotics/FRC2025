// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve.*;
import frc.robot.subsystems.vision.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Vision vision;
  private final Drive drive;
  //private final SendableChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> autoChooser1;
  private final CommandPS5Controller controller = new CommandPS5Controller(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {



    switch (SimConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(limelightCamera, drive::getRotation),
                new VisionIOPhotonVision(camera0Name, robotToCamera1));
                //new VisionIOPhotonVision(camera1Name, robotToCamera2));
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
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera1, drive::getPose));
                //new VisionIOPhotonVisionSim(camera1Name, robotToCamera2, drive::getPose));
        break;

      default:

        drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    autoChooser1 = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());


    autoChooser1.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser1.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser1.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser1.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser1.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser1.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the trigger bindings
    configureBindings();


    //autoChooser = AutoBuilder.buildAutoChooser();
  }

 
  private void configureBindings() {

        drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .triangle()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.cross().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .square()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    Pose2d currentPose = drive.getPose();
    Pose2d startPose = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    Pose2d endPose =new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0)), new Rotation2d());

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);
    PathPlannerPath path = new PathPlannerPath(
      waypoints, 
      new PathConstraints(2.0, 2.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)), 
      null, 
      new GoalEndState(0.0, currentPose.getRotation()));

    controller.R1().onTrue(AutoBuilder.followPath(path));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    //return drive.followPathCommand("Example Path");s
    return autoChooser1.get();
    
  }
}
