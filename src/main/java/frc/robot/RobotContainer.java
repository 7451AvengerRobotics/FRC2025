// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ButtonConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FullIntakeCommand;
import frc.robot.commands.LEDCommands.setLedColorCommand;
import frc.robot.generated.TunerConstantsNew;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedHandler;
import frc.robot.subsystems.Swerve.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
  //private final Vision vision;
  private final Drive drive;
  //private final SendableChooser<Command> autoChooser;
  private final CommandPS5Controller controller = new CommandPS5Controller(1);
  private final Joystick buttonPannel = new Joystick(0);

  private final LoggedDashboardChooser<Command> autoChooser;

  // Subsytems
  private final Intake intake = new Intake();
  private final Claw claw = new Claw();
  private final Elevator elevator = new Elevator();
  private final Index index = new Index();
  private final LedHandler led = new LedHandler();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    switch (SimConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstantsNew.FrontLeft),
                new ModuleIOTalonFX(TunerConstantsNew.FrontRight),
                new ModuleIOTalonFX(TunerConstantsNew.BackLeft),
                new ModuleIOTalonFX(TunerConstantsNew.BackRight));
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         //new VisionIOPhotonVision(limelightCamera, robotToCamera2)
        //         new VisionIOPhotonVision(camera0Name, robotToCamera1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(TunerConstantsNew.FrontLeft),
                  new ModuleIOSim(TunerConstantsNew.FrontRight),
                  new ModuleIOSim(TunerConstantsNew.BackLeft),
                  new ModuleIOSim(TunerConstantsNew.BackRight));
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(camera0Name, robotToCamera1, drive::getPose),
        //         new VisionIOPhotonVisionSim(camera1Name, robotToCamera2, drive::getPose));
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
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;


    }

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


    // Configure the trigger bindings
    configureBindings();

    led.setDefaultCommand(new setLedColorCommand(led, 255, 0, 0));


  }

 
  private void configureBindings() {

      JoystickButton processor = new JoystickButton(buttonPannel, ButtonConstants.processor);
      JoystickButton L4 = new JoystickButton(buttonPannel, ButtonConstants.L4);
      JoystickButton L3 = new JoystickButton(buttonPannel, ButtonConstants.L3);
      JoystickButton L2 = new JoystickButton(buttonPannel, ButtonConstants.L2);
      JoystickButton algae2 = new JoystickButton(buttonPannel, ButtonConstants.algae2);
      JoystickButton intakeAlgae = new JoystickButton(buttonPannel, ButtonConstants.intakeAlgae);
      JoystickButton reset = new JoystickButton(buttonPannel, ButtonConstants.reset);
      JoystickButton blueReef = new JoystickButton(buttonPannel, ButtonConstants.blueReef);
      JoystickButton greenReef = new JoystickButton(buttonPannel, ButtonConstants.greenReef);
      JoystickButton redReef = new JoystickButton(buttonPannel, ButtonConstants.redReef);
      JoystickButton whiteReef = new JoystickButton(buttonPannel, ButtonConstants.whiteReef);
      JoystickButton yellowReef = new JoystickButton(buttonPannel, ButtonConstants.yellowReef);

      boolean intakeAll = buttonPannel.getRawAxis(ButtonConstants.intake1Axis) > 0;

      if (intakeAll) {
        new FullIntakeCommand(intake, index, claw, 1, 0.7, 0.2);
      }

      claw.setDefaultCommand(claw.setClawPivotAngle(0));

        drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
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
    //controller.PS().onTrue(Commands.runOnce(drive::stopWithX, drive));

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

    // controller.R1().whileTrue(drive.driveToPose(AllianceFlipUtil.apply(Processor.centerFace.plus(new Transform2d(new Translation2d(1, 0), new Rotation2d(0))))));
    // controller.cross().whileTrue(drive.driveToPose(AllianceFlipUtil.apply(Reef.reef0.plus(new Transform2d(new Translation2d(0.5,0), new Rotation2d(0))))));
    // controller.triangle().whileTrue(drive.driveToPose(AllianceFlipUtil.apply(Reef.reef3.plus(new Transform2d(new Translation2d(0.5,0), new Rotation2d(0))))));

    controller.L1().whileTrue(Commands.runOnce(() -> {
      Pose2d currentPose = drive.getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(0, 0.1)), new Rotation2d());

      //List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos); 
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, new Pose2d(currentPose.getTranslation().plus(new Translation2d(0, 0.05)), new Rotation2d()), endPos);
      PathPlannerPath path = new PathPlannerPath(
        waypoints, 
        new PathConstraints(
          2, 1, 
          Units.degreesToRadians(360), Units.degreesToRadians(90)
        ),
        null, // Ideal starting state can be null for on-the-fly paths
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule(); 
    }));

    controller.R1().whileTrue(Commands.runOnce(() -> {
      Pose2d currentPose = drive.getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(0, -0.1)), new Rotation2d());

      //List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos); 
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, new Pose2d(currentPose.getTranslation().plus(new Translation2d(0, 0.05)), new Rotation2d()), endPos);
      PathPlannerPath path = new PathPlannerPath(
        waypoints, 
        new PathConstraints(
          2, 1, 
          Units.degreesToRadians(360), Units.degreesToRadians(90)
        ),
        null, // Ideal starting state can be null for on-the-fly paths
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule(); 
    }));

    controller.PS().whileTrue(new ParallelCommandGroup(intake.setintakePower(1), index.setIndexPower(0.7), claw.setClawPower(0.2)));
    controller.circle().onTrue(new SequentialCommandGroup(claw.setClawPivotAngle(0).andThen(elevator.setElevatorPosition(2))));
    controller.L1().onTrue(new SequentialCommandGroup(elevator.setElevatorPosition(0)));
    controller.R1().onTrue(new ParallelCommandGroup(claw.setClawPivotAngle(0), (elevator.setElevatorPosition(5.58))));
    controller.cross().onTrue(intake.setIntakePivotAngle(0.246));

    controller.R2().whileTrue(new FullIntakeCommand(intake, index, claw, 0.7, 1, 0.4));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return drive.followPathCommand("Example Path");
    return autoChooser.get();
  }
}
