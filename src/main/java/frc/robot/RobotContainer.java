// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ButtonConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Autos.AutoRoutines;
import frc.robot.commands.LEDCommands.LedStrobeCommand;
import frc.robot.commands.LEDCommands.setLedColorCommand;
import frc.robot.generated.TunerConstantsNew;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.LedHandler;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawPivot;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Swerve.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;

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
  private final CommandPS5Controller manip = new CommandPS5Controller(2);
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
  JoystickButton intakeTrough = new JoystickButton(buttonPannel, ButtonConstants.intakeTrough);
  JoystickButton L4 = new JoystickButton(buttonPannel, ButtonConstants.L4);
  JoystickButton L3 = new JoystickButton(buttonPannel, ButtonConstants.L3);
  JoystickButton L2 = new JoystickButton(buttonPannel, ButtonConstants.L2);
  JoystickButton intakeAlgae = new JoystickButton(buttonPannel, ButtonConstants.intakeAlgae);

  Trigger L2req = new Trigger(L2::getAsBoolean);
  Trigger L3req = new Trigger(L3::getAsBoolean);
  Trigger L4req = new Trigger(L4::getAsBoolean);
  Trigger scoreReq = new Trigger(intakeAlgae::getAsBoolean);
  Trigger L1req = new Trigger(intakeTrough::getAsBoolean);
  Trigger stowClaw = new Trigger(claw::clawBroke);
  //Trigger holdingBall = new Trigger(claw::motorStall);

  public final SuperStructure superStructure;
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
            new VisionIOPhotonVision(VisionConstants.limelight1Camera, VisionConstants.limelight2Transform3d)
        );

        superStructure = new SuperStructure(
            intake, 
            intakePivot, 
            index, 
            elevator, 
            clawPivot, 
            claw, 
            climb
        );

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

        superStructure = new SuperStructure(
            intake, 
            intakePivot, 
            index, 
            elevator, 
            clawPivot, 
            claw, 
            climb
        );
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

        superStructure = new SuperStructure(
            intake, 
            intakePivot, 
            index, 
            elevator, 
            clawPivot, 
            claw, 
            climb
        );

        break;

    }

    autos = new AutoRoutines(drive, elevator, clawPivot, intakePivot, claw, intake, index);
    autoChooser1 = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser1);
    

    configureBindings();
    configAutos();
    

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
                claw::notClawBroke)
    );
  }

  private void configureBindings() {

    stowClaw.onTrue(superStructure.stow());

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * 0.85,
            () -> -controller.getLeftX() * 0.85,
            () -> -controller.getRightX() * 0.7));

    // Switch to X pattern when X button is pressed
    controller.PS().and(controller.touchpad()).onTrue(
        Commands.runOnce(
            drive::stopWithX,
            drive
        )
    );

    controller.options().and(controller.PS()).onTrue(
        Commands.runOnce(
            () -> drive.setPose(
                new Pose2d(
                    drive.getPose().getTranslation(),
                    new Rotation2d())),
            drive)
            .ignoringDisable(true));
    

    controller.R2().onTrue(superStructure.intake().andThen(superStructure.stow()));     
    
    controller.R1().whileTrue(
        Commands.sequence(
            Commands.parallel(
                drive.driveToClosestReefScoringFaceWithTranslate(
                    new Transform2d(new Translation2d(0.52, 0.15), new Rotation2d())),
                superStructure.setReefLvl(manip)
            ),
            Commands.waitUntil(manip.touchpad()),
            superStructure.score().until(claw::notClawBroke).andThen(drive.driveToClosestReefScoringFaceWithTranslate(
                new Transform2d(new Translation2d(0.65, 0.15), new Rotation2d())))
        )
    ).onFalse(
        superStructure.stow()
    );

    controller.L1().whileTrue(
        Commands.sequence(
            Commands.parallel(
                drive.driveToClosestReefScoringFaceWithTranslate(
                    new Transform2d(new Translation2d(0.52, -0.21), new Rotation2d())
                ),
                superStructure.setReefLvl(manip)
            ),
            Commands.waitUntil(manip.touchpad()),
            superStructure.score().until(claw::notClawBroke).andThen(drive.driveToClosestReefScoringFaceWithTranslate(
                new Transform2d(new Translation2d(0.65, -0.21), new Rotation2d())))
        )
    ).onFalse(
        superStructure.resetEverything()
    );

    controller.touchpad().whileTrue(
        Commands.sequence(
            Commands.parallel(
                drive.driveToClosestReefScoringFaceWithTranslate(
                    new Transform2d(new Translation2d(0.48, 0), new Rotation2d(0))
                ),
                superStructure.setAlgaeLvl(manip)
            )
        )
    ).onFalse(
        superStructure.resetEverything()
    );

    controller.triangle().onTrue(
        superStructure.setL2Algae()
    );

    controller.square().onTrue(superStructure.resetEverything());

    controller.cross().onTrue(
        superStructure.setL3Algae()
    );

    controller.circle().onTrue(
        superStructure.setL4()
    );

    controller.povDown().onTrue(
        superStructure.readyToClimb()
    );

    controller.povUp().onTrue(
        superStructure.climb()
    );

    controller.L2().whileTrue(
        superStructure.score()
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser1.getSelected();
  }

  public void configAutos() {
    autoChooser1.addOption("Processor Side L4 Coral ", autos.processorSide2L4Coral());
    autoChooser1.addOption("Processor Side L2 Coral ", autos.processorSide2L2Coral());
    autoChooser1.addOption("Processor Red Side L2 Coral ", autos.processorRedSide2L2Coral());
    autoChooser1.addOption("Barge Score 1 L2 1 L4", autos.bargeSide2L2Coral());
    autoChooser1.addOption("Processor Side 3 L2", autos.processorSide3L2Coral());
    autoChooser1.addOption("red Test", autos.redTest());
    autoChooser1.addOption("Center Auto", autos.centerAuto());
  }
}
