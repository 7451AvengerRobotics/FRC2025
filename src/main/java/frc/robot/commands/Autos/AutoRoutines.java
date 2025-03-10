package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve.Drive;

public class AutoRoutines {
    private final Drive drive;

    public AutoRoutines(Drive drive) {
        this.drive = drive;
    }

    public Command processorSide2Coral() {
        return Commands.sequence(
                drive.followPPPathCommand("InitPath"),
                drive.driveToClosestReefScoringFaceWithTranslate(
                        new Transform2d(new Translation2d(0.52, 0.18), new Rotation2d(0))),
                drive.followPPPathCommand("Source"),
                drive.followPPPathCommand("BackReef"),
                drive.driveToClosestReefScoringFaceWithTranslate(
                        new Transform2d(new Translation2d(0.52, 0.18), new Rotation2d(0))));
    }
}