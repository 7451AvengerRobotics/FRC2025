
package frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.util.AllianceFlipUtil;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Swerve.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Rotation2d m_desiredRot;
    private double kPTheta = 5;    
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();
        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;
    private final PIDController m_thetaController = new PIDController(kPTheta, 0.0, 0.0);




    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

    
    }



    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0.3951224;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()->{
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

public Supplier<Rotation2d> angleToSpeakerSupplier(final Supplier<Pose2d> currentPoseSupplier) {

    // Translation2d targetTranslation = getPose().getTranslation().minus(FieldConstants.getSpeakerPose().getTranslation());

    // double x = targetTranslation.getX();

    // double y = targetTranslation.getY();

    // double angle = Math.atan(y/x);

    


        return () -> currentPoseSupplier.get()
                .getTranslation()
                .minus(FieldConstants.getSpeakerPose().getTranslation())
                .getAngle();
    }



    public Command faceAngle(final Supplier<Rotation2d> rotationTargetSupplier) {
        return 
                run(() -> {
                    m_desiredRot = AllianceFlipUtil.apply(rotationTargetSupplier.get());
                    var curPose = getState().Pose;
                    var thetaSpeed = m_thetaController.calculate(curPose.getRotation().getRadians(), 
                                                                    rotationTargetSupplier.get().getRadians());
                    var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, thetaSpeed, m_desiredRot);
                   
                   setControl(AutoRequest.withSpeeds(speeds));
                });
    }

    public Pose2d getPose(){
        return getState().Pose;
    }





     public Command pathfindToPose(Pose2d endPose, PathConstraints pathConstraints, double endVelocity) {
        return AutoBuilder.pathfindToPose(endPose, pathConstraints, endVelocity);
    }

    public Rotation2d faceAnySideOfRobotInDirectionOfTravel(PathPlannerTrajectory.State state) {
        var currentHolonomicRotation = getState().Pose.getRotation();
        var travelDirection = state.heading;

        var diff = travelDirection.minus(currentHolonomicRotation);

        var diffDegrees = diff.getDegrees() % 360; // -360 to 360
        while (diffDegrees < -45) {
            diffDegrees += 90;
        }
        while (diffDegrees > 45) {
            diffDegrees -= 90;
        }
        // diffDegrees between -45 to 45;

        return Rotation2d.fromDegrees(currentHolonomicRotation.getDegrees() % 360 + diffDegrees);
    }

    public Rotation2d faceFrontTowardsRobotDirectionOfTravel(PathPlannerTrajectory.State state) {
        var travelDirection = state.heading;

        return Rotation2d.fromDegrees(travelDirection.getDegrees() % 360);
    }

    public Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }


    public Command aim(double radians) {
		return run(() -> {
			m_desiredRot = AllianceFlipUtil.apply(Rotation2d.fromRadians(radians));
			var curPose = getState().Pose;
			var thetaSpeed = m_thetaController.calculate(curPose.getRotation().getRadians(),
				m_desiredRot.getRadians());
			var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, thetaSpeed, m_desiredRot);

			setControl(AutoRequest.withSpeeds(speeds));
		}).until(() -> {
			boolean check = MathUtil.isNear(m_desiredRot.getDegrees(), getState().Pose.getRotation().getDegrees(), 1);
			if (check) {
				System.out.println("it should end");
			}
			return check;
		});
	}

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective  = true;
            });

            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

             this.addVisionMeasurement(limelightMeasurement.pose, (LimelightHelpers.getLatency_Pipeline("limelight")/1000.0) - 
                 (LimelightHelpers.getLatency_Capture("limelight")/1000.0), VecBuilder.fill(0.9, 0.9, 0.9));

            
            SmartDashboard.putNumber("Vision Pose X", Eyes.getRobotPose().getX());
            SmartDashboard.putNumber("Vision Pose y", Eyes.getRobotPose().getY());
            SmartDashboard.putNumber("Vision Pose Rot", Eyes.getRobotPose().getRotation().getRadians());
        }
    }
}
