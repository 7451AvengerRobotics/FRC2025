// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Autos.AutoRoutines;
import frc.robot.commands.LEDCommands.LedStrobeCommand;
import frc.robot.commands.LEDCommands.setLedColorCommand;
import frc.robot.generated.TunerConstantsNew;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.LedHandler;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawPivot;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Swerve.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.GamepadAxisButton;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.vision.VisionIO;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Vision vision;
  private final Drive drive;
  private final CommandPS5Controller controller = new CommandPS5Controller(1);
  private final Joystick buttonPannel = new Joystick(0);
  private final SendableChooser<Command> autoChooser1;
  private final AutoRoutines autos;

  // Subsytems
  private final Intake intake = new Intake();
  private final IntakePivot intakePivot = new IntakePivot();
  private final Claw claw = new Claw();
  private final ClawPivot clawPivot = new ClawPivot();
  private final Elevator elevator = new Elevator();
  private final Index index = new Index();
  private final LedHandler led = new LedHandler();
  private final Climber climb = new Climber();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // NamedCommands.registerCommand("L2",
    // elevator.setElevatorPosition(2.1).until(elevator::endCommand).andThen(claw.setClawPower(0.3)));

    switch (SimConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstantsNew.FrontLeft),
            new ModuleIOTalonFX(TunerConstantsNew.FrontRight),
            new ModuleIOTalonFX(TunerConstantsNew.BackLeft),
            new ModuleIOTalonFX(TunerConstantsNew.BackRight));
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.frontLeftTransform3d),
            new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.frontRightTransform3d),
            new VisionIOPhotonVision(VisionConstants.limelight2Camera, VisionConstants.limelight3Transform3d),
            new VisionIOPhotonVision(VisionConstants.limelight1Camera, VisionConstants.limelight2Transform3d));
        // VisionConstants.limelight2Transform3d));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(TunerConstantsNew.FrontLeft),
            new ModuleIOSim(TunerConstantsNew.FrontRight),
            new ModuleIOSim(TunerConstantsNew.BackLeft),
            new ModuleIOSim(TunerConstantsNew.BackRight));
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.frontLeftTransform3d,
                drive::getPose));
        break;

      default:

        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
        }, new VisionIO() {
        });
        break;

    }

    autos = new AutoRoutines(drive, elevator, clawPivot, intakePivot, claw, intake, index);
    autoChooser1 = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser1);
    // NamedCommands.registerCommand("Score",
    // drive.driveToClosestReefScoringFaceWithTranslate(new Transform2d(new
    // Translation2d(0,0.18), new Rotation2d(0))));
    // NamedCommands.registerCommand("Score", claw.setClawPower(0.5));
    NamedCommands.registerCommand("Score", 
    Commands.sequence(
    clawPivot.setClawPivotAngle(0.03).until(clawPivot::clawClear),
    Commands.parallel(
            drive.driveToClosestReefScoringFaceWithTranslate(
                    new Transform2d(new Translation2d(0.52, -0.18),
                            new Rotation2d(0))),
            elevator.setElevatorPosition(1.9)
                    .until(
                            elevator::endCommand)
                    .onlyIf(
                            clawPivot::clawClear))
            .andThen(claw.setClawPower(0.4).until(claw::notClawBroke))));

    configureBindings();
    autoChooser1.addOption("Processor Side L4 Coral ", autos.processorSide2L4Coral());
    autoChooser1.addOption("Processor Side L2 Coral ", autos.processorSide2L2Coral());
    autoChooser1.addOption("Processor Red Side L2 Coral ", autos.processorRedSide2L2Coral());
    autoChooser1.addOption("Barge Score 1 L2 1 L4", autos.bargeSide2L2Coral());
    autoChooser1.addOption("Processor Side 3 L2", autos.processorSide3L2Coral());
    autoChooser1.addOption("red Test", autos.redTest());
    autoChooser1.addOption("Center Auto", autos.centerAuto());

    led.setDefaultCommand(
        new setLedColorCommand(led, 255, 255, 0)
            .until(
                intake::getIntakeBreak)
            .andThen(
              new setLedColorCommand(led, 255, 255, 0))
            .until(
                claw::clawBroke)
            .andThen(
                new LedStrobeCommand(led, false).withTimeout(1))
            .andThen(
                new setLedColorCommand(led, 0, 255, 0))
            .until(
                claw::notClawBroke));
  }

  private void configureBindings() {

    JoystickButton processor = new JoystickButton(buttonPannel, ButtonConstants.processor);
    JoystickButton intakeTrough = new JoystickButton(buttonPannel, ButtonConstants.intakeTrough);
    JoystickButton L4 = new JoystickButton(buttonPannel, ButtonConstants.L4);
    JoystickButton L3 = new JoystickButton(buttonPannel, ButtonConstants.L3);
    JoystickButton L2 = new JoystickButton(buttonPannel, ButtonConstants.L2);
    // JoystickButton algae2 = new JoystickButton(buttonPannel,
    // ButtonConstants.algae2);
    JoystickButton intakeAlgae = new JoystickButton(buttonPannel, ButtonConstants.intakeAlgae);
    JoystickButton reset = new JoystickButton(buttonPannel, ButtonConstants.reset);
    JoystickButton blueReef = new JoystickButton(buttonPannel, ButtonConstants.blueReef);
    JoystickButton greenReef = new JoystickButton(buttonPannel, ButtonConstants.greenReef);
    JoystickButton redReef = new JoystickButton(buttonPannel, ButtonConstants.redReef);
    JoystickButton whiteReef = new JoystickButton(buttonPannel, ButtonConstants.whiteReef);
    JoystickButton yellowReef = new JoystickButton(buttonPannel, ButtonConstants.yellowReef);
    GamepadAxisButton L1 = new GamepadAxisButton(this::axis1ThresholdLessThanPoint5);
    // GamepadAxisButton algae1 = new
    // GamepadAxisButton(this::axis0ThresholdGreatererThanPoint5);
    // GamepadAxisButton blackReef = new
    // GamepadAxisButton(this::axis0ThresholdLessThanPoint5);

    clawPivot.setDefaultCommand(
        clawPivot.setClawPivotAngle(0.03));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * 0.7,
            () -> -controller.getLeftX() * 0.7,
            () -> -controller.getRightX() * 0.7));

    // Switch to X pattern when X button is pressed
    controller.PS().onTrue(
        Commands.runOnce(
            drive::stopWithX,
            drive));

    controller.options().onTrue(clawPivot.setClawPivotAngle(-0.061));

    // Drive Commands
    controller.R1().whileTrue(
        drive.driveToClosestReefScoringFaceWithTranslate(
            new Transform2d(new Translation2d(0.52, 0.15), new Rotation2d())));

    controller.L1().whileTrue(
        drive.driveToClosestReefScoringFaceWithTranslate(
            new Transform2d(new Translation2d(0.52, -0.21), new Rotation2d())));

    controller.touchpad().whileTrue(
        drive.driveToClosestReefScoringFaceWithTranslate(
            new Transform2d(new Translation2d(0.48, 0), new Rotation2d(0))));

    processor.whileTrue(Commands.parallel(
        drive.driveToPose(
            FieldConstants.Processor.centerFace.plus(new Transform2d(new Translation2d(0.58, 0), new Rotation2d(0))))));

    intakeAlgae.whileTrue(claw.setClawPower(0.5));

    // Reset gyro to 0° when B button is pressed
    controller.square().onTrue(
        Commands.runOnce(
            () -> drive.setPose(
                new Pose2d(
                    drive.getPose().getTranslation(),
                    new Rotation2d())),
            drive)
            .ignoringDisable(true));

    // Score or Suck
    controller.L2().whileTrue((claw.setClawPower(0.4)));

    // Climber One
    controller.povDown().onTrue(
        Commands.parallel(
            intakePivot.setIntakePivotAngle(0.36),
            clawPivot.setClawPivotAngle(0),
            climb.setClimberAngle(-0.22)));
    controller.povUp().whileTrue(
        Commands.parallel(
            clawPivot.setClawPivotAngle(0),
            climb.setClimberPower(0.37)
                .until(climb::endClimbSeq)));

    // L1 - L4
    L1.whileTrue(
        Commands.parallel(
            intakePivot.setIntakePivotAngle(0.15)
                .until(
                    intakePivot::endCommand)
                .andThen(
                    intake.setintakePower(-0.23))));

    L2.onTrue(
        (elevator.setElevatorPosition(1.9)
            .onlyIf(
                clawPivot::clawClear)));

    blueReef.onTrue(Commands.parallel(elevator.setElevatorPosition(1.4), clawPivot.setClawPivotAngle(0.09)));
    greenReef.onTrue(Commands.parallel(elevator.setElevatorPosition(3), clawPivot.setClawPivotAngle(0.09)));

    L3.onTrue(
        (elevator.setElevatorPosition(3.35)
            .onlyIf(
                clawPivot::clawClear)));

    L4.onTrue(
        elevator.setElevatorPosition(5.5)
            .until(
                elevator::endCommand)
            .onlyIf(
                clawPivot::clawClear)
            .andThen(
                clawPivot.setClawPivotAngle(-0.008)));

    yellowReef.whileTrue(
        drive.driveToClosestReefScoringFaceWithTranslate(
            new Transform2d(new Translation2d(0.56, 0.15), new Rotation2d(0))));

    whiteReef.whileTrue(
        drive.driveToClosestReefScoringFaceWithTranslate(
            new Transform2d(new Translation2d(0.56, -0.18), new Rotation2d(0))));

    redReef.onTrue(
        Commands.parallel(
            intake.setintakePower(-0.5),
            index.setIndexPower(-0.7),
            claw.setClawPower(0.1),
            clawPivot.setClawPivotAngle(0.03),
            intakePivot.setIntakePivotAngle(.2)).withTimeout(0.5)
            .andThen(
                Commands.parallel(
                    intake.setintakePower(0.5),
                    index.setIndexPower(0.7),
                    claw.setClawPower(0.2),
                    clawPivot.setClawPivotAngle(-0.06)))
            .until(claw::clawBroke));
    // Reset
    reset.onTrue(
        climb.setClimberAngle(0)
            .until(
                climb::endClimbCommand)
            .andThen(
                Commands.parallel(
                    intakePivot.setIntakePivotAngle(0.06),
                    elevator.setElevatorPosition(0.0002),
                    clawPivot.setClawPivotAngle(0.03))));

    // Intake All
    controller.R2().onTrue(
        Commands.parallel(
            intake.setintakePower(1),
            intakePivot.setIntakePivotAngle(0.36))
            .until(intake::getIntakeBreak)
            .andThen(
                Commands.parallel(
                    intake.setintakePower(0.5),
                    index.setIndexPower(0.7),
                    claw.setClawPower(0.1),
                    clawPivot.setClawPivotAngle(-0.058),
                    intakePivot.setIntakePivotAngle(.2)))
            .until(claw::clawBroke)
            .andThen(
                Commands.parallel(
                    intakePivot.setIntakePivotAngle(0),
                    clawPivot.setClawPivotAngle(0.03))));

    

    
    // Intake Trough
    intakeTrough.onTrue(
        Commands.parallel(
            intakePivot.setIntakePivotAngle(0.38),
            intake.setintakePower(0.7))
            .until(intake::getIntakeBreak));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser1.getSelected();
  }

  public boolean axis1ThresholdGreatererThanPoint5() {
    return buttonPannel.getRawAxis(1) > .5;
  }

  public boolean axis0ThresholdGreatererThanPoint5() {
    return buttonPannel.getRawAxis(0) > 0;
  }

  public boolean axis0ThresholdLessThanPoint5() {
    return buttonPannel.getRawAxis(0) < 0;
  }

  public boolean axis1ThresholdLessThanPoint5() {
    return buttonPannel.getRawAxis(1) < .5;
  }
}
