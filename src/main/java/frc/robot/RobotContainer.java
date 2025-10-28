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
import frc.robot.util.ControllerUtil;
import frc.robot.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Set;

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
  public final CommandPS5Controller manip = new CommandPS5Controller(2);
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

  Trigger stowClaw = new Trigger(claw::clawBroke);
  Trigger stallClaw = new Trigger(claw::motorStall);
  Trigger eleLow = new Trigger(elevator::dontStallClaw);
  Trigger realRobot = new Trigger(Robot::isReal);
  Trigger endgameTrigger = new Trigger(() -> DriverStation.getMatchTime() <= 20)
            .and(DriverStation::isFMSAttached)
            .and(RobotModeTriggers.teleop());
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
            new VisionIOPhotonVision(VisionConstants.backRight, VisionConstants.backRightTransform3d),
            new VisionIOPhotonVision(VisionConstants.backLeft, VisionConstants.backLeftTransform3d)
            //new VisionIOPhotonVision(VisionConstants.frontRight, VisionConstants.frontRightTransform3d)
            //new VisionIOPhotonVision(VisionConstants.limelight1Camera, VisionConstants.limelight2Transform3d)
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
            new VisionIOPhotonVisionSim(VisionConstants.backRight, VisionConstants.backRightTransform3d,
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

    autos = new AutoRoutines(drive, elevator, clawPivot, intakePivot, claw, intake, index, superStructure);
    autoChooser1 = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser1);
    

    configureBindings();
    configAutos();

//claw.setDefaultCommand(claw.setClawPower(0).until(claw::motorStall).andThen(claw.setClawPower(-0.3)).andThen(new setLedColorCommand(led, 0, 255, 255)));
  }

  private void configureBindings() {
    endgameTrigger.onTrue(ControllerUtil.rumbleForDurationCommand(
                controller.getHID(), GenericHID.RumbleType.kBothRumble, 0.5, 1)
        );

    // stowClaw.onTrue(superStructure.stow());
    stallClaw.and(eleLow.negate()).whileTrue(new setLedColorCommand(led, 0, 255, 200));
    led.setDefaultCommand((
        new setLedColorCommand(led, 255, 0, 0)
            .until(
                intake::getIntakeBreak)
            .andThen(
              new setLedColorCommand(led, 0, 255, 0))
            .until(
                claw::clawBroke)
            .andThen(
                new LedStrobeCommand(led, false).withTimeout(1))
            .andThen(
                new setLedColorCommand(led, 255, 255, 255))
            .until(
                claw::notClawBroke)
        ));
    

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * 0.85,
            () -> -controller.getLeftX() * 0.85,
            () -> -controller.getRightX() * 0.7
        )
    );

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
                    new Rotation2d()
                )
            ),
            drive
        ).ignoringDisable(true)
    );
    

    // intake
    controller.R2().onTrue(
        Commands.defer(
            ()-> Commands.either(superStructure.intakeDown(), superStructure.intakeWithBall(), stallClaw.negate()),
            Set.of(intake))
            .andThen(
                superStructure.resetEverything()
            )
            .andThen(
                new setLedColorCommand(led, 255, 255, 255)
            )
    );

    manip.L2().onTrue(superStructure.intakeWithBall());

    // fix intake
    manip.triangle().onTrue(
        superStructure.fixIntake().andThen(superStructure.resetEverything())
    );

    
    // right side score
    controller.R1().onTrue(
        Commands.sequence(
            Commands.parallel(
                drive.driveToClosestReefScoringFaceWithTranslate(
                    new Transform2d(new Translation2d(0.64, 0.13), 
                    new Rotation2d()
                ),
                manip).withTimeout(2),
                superStructure.setReefLvl()
            ),
            Commands.waitUntil(manip.R2()),
            superStructure.score()
        )
    ).onFalse(
        Commands.sequence(
            Commands.waitUntil(controller.axisMagnitudeGreaterThan(0, 0.2)),
            superStructure.resetEverything()
        )
    );

    // left side score
    controller.L1().whileTrue(
        Commands.sequence(
            Commands.parallel(
                drive.driveToClosestReefScoringFaceWithTranslate(
                    new Transform2d(
                        new Translation2d(0.64, -0.21),
                        new Rotation2d()
                    ),
                    manip
                ).withTimeout(2),
                superStructure.setReefLvl()
            ),
            Commands.waitUntil(manip.R2()),
            superStructure.score()
        )
    ).onFalse(
        Commands.sequence(
            Commands.waitUntil(controller.axisMagnitudeGreaterThan(0, 0.2)),
            superStructure.resetEverything()
        )
    );

    // Algae
    controller.touchpad().whileTrue(
            Commands.defer(() -> 
                Commands.parallel(
                    drive.driveToClosestReefScoringFaceWithTranslate(
                        new Transform2d(
                            new Translation2d(0.61, 0),
                            new Rotation2d()
                        ),
                        manip
                    ).withTimeout(2),
                    superStructure.autoSetAlgaeHeight(drive.getClosestReefFace())),
                    Set.of(drive)
            ).andThen(claw.setClawPower(-0.5))
    ).onFalse(
        Commands.defer(
            () -> superStructure.autoSetAlgaeHeight(
                drive.getClosestReefFace()  
            ),
            Set.of(claw))
    );

        //barge
    manip.square().whileTrue(
        Commands.sequence(
            Commands.parallel(
                drive.driveFromCurrentPose(7.75),
                superStructure.setBargeAlgae()
            ),
        Commands.waitUntil(manip.R2()),
            superStructure.outtakeAlgae()
        )
    ).onFalse(
        Commands.sequence(
            Commands.waitUntil(controller.axisMagnitudeGreaterThan(0, 0.1)),
            superStructure.resetEverything()
        )
    );

    manip.cross().onTrue(
        Commands.sequence(
        superStructure.setBargeAlgae(),
        Commands.waitUntil(manip.R2()),
            superStructure.outtakeAlgae()
        )
    ).onFalse(
        Commands.sequence(
            Commands.waitUntil(controller.axisMagnitudeGreaterThan(0, 0.2)),
            superStructure.resetEverything()
        )
    );

    manip.PS().onTrue(
        claw.setClawPower(-0.3)
    );
    
    manip.circle().onTrue(
        superStructure.setProcessor()
    );

    controller.circle().onTrue(
        superStructure.setReefLvl()
    );

    controller.square().onTrue(
        superStructure.resetEverything()
    );

    controller.povDown().onTrue(
        superStructure.readyToClimb()
    );

    controller.povUp().onTrue(
        superStructure.climb()
    );

    controller.L2().whileTrue(  
        Commands.either(
            superStructure.score(),
            superStructure.outtakeAlgae(),
            claw::clawBroke)
    );

    manip.touchpad().whileTrue(
        Commands.sequence(
            Commands.parallel(
                drive.driveToClosestReefScoringFaceWithTranslate(
                    new Transform2d(
                        new Translation2d(0.57, 0),
                        new Rotation2d()
                    ),
                    manip
                ),
                superStructure.setL1()
            ),
            Commands.waitUntil(manip.R2()),
            superStructure.scoreL1()
        )
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
    autoChooser1.addOption("BargeLoli", autos.bargeSideLoli());
    autoChooser1.addOption("ProcessorLoli", autos.processorSideLoli());
    autoChooser1.addOption("ProcessorIALoli", autos.processorIALoli());

  }
}
