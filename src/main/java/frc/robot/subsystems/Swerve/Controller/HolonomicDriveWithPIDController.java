package frc.robot.subsystems.Swerve.Controller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicDriveWithPIDController {

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    public HolonomicDriveWithPIDController(
        final PIDController xController,
        final PIDController yController,
        final PIDController rotationController,
        final Pose2d poseTolerance
    ) {
        this.xController = xController;
        this.xController.setTolerance(poseTolerance.getX(), poseTolerance.getX() * 1.5);

        this.yController = yController;
        this.yController.setTolerance(poseTolerance.getY(), poseTolerance.getX() * 1.5);

        this.rotationController = rotationController;
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset(final Pose2d currentPose, final ChassisSpeeds robotRelativeSpeeds) {
        xController.reset();
        yController.reset();
        rotationController.reset();
    }

    public boolean atReference() {
        return xController.atSetpoint()
                && yController.atSetpoint()
                && rotationController.atSetpoint();
    }

    public void setTolerance(final Pose2d poseTolerance) {
        this.xController.setTolerance(poseTolerance.getX(), poseTolerance.getX() * 1.5);
        this.yController.setTolerance(poseTolerance.getY(), poseTolerance.getY() * 1.5);
    }

    public ChassisSpeeds calculate(final Pose2d currentPose, final Pose2d targetPose) {
        final double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
        final double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());

        final double rotationFeedback = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
        );

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFeedback,
                yFeedback,
                rotationFeedback,
                currentPose.getRotation()
        );

        
    }
    
}
