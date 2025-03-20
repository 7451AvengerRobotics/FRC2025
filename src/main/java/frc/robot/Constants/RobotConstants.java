package frc.robot.Constants;

public class RobotConstants {

    public static class IndexConstants{
        public static final int kIndexID = 51;
    }
    public static class IntakeConstants{
        public static final int kIntakePivotID = 49;
        public static final int kIntakeID = 41;
        public static final double kIntakeGearRatio = 62.5;
        public static final double intakePivotkS = 0.11518; // Add 0.25 V output to overcome static friction
        public static final double intakePivotkV = 6.6; // A velocity target of 1 rps results in 0.12 V output
        public static final double intakePivotkA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double intakePivotkP = 60; // A position error of 0.2 rotations results in 12 V output
        public static final double intakePivotkI = 0; // No output for integrated error
        public static final double intakePivotkD = 1; 
        public static final double intakePivotkG = 0.016; 

    }
    public static class ClawPivotConstants{
        public static final int kClawPivotID = 33;
        public static final double kClawPivotGearRatio = 25;
        public static final double clawPivotkS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double clawPivotkV = 3; // A velocity target of 1 rps results in 0.12 V output
        public static final double clawPivotkA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double clawPivotkP = 50; // A position error of 0.2 rotations results in 12 V output
        public static final double clawPivotkI = 0; // No output for integrated error
        public static final double clawPivotkD = 0; 
        public static final double clawPivotkG = 0.17; 
    }

    public static class ClawConstants{
        public static final int kClawID = 27;
        public static final double kClawGearRatio = 10;
        public static final double clawkS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double clawkV = 3; // A velocity target of 1 rps results in 0.12 V output
        public static final double clawkA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double clawkP = 5; // A position error of 0.2 rotations results in 12 V output
        public static final double clawkI = 0; // No output for integrated error
        public static final double clawkD = 0; 
        public static final double clawkG = 0; 
    }
    
    public static class ElevatorConstants {
        public static final int kElevatorID = 35;
        public static final double kElevatorGearRatio = 5;
        public static final double elevatorkS = 0.144; // Add 0.25 V output to overcome static friction
        public static final double elevatorkV = 0.6; // A velocity target of 1 rps results in 0.12 V output
        public static final double elevatorkA = 0;// An acceleration of 1 rps/s requires 0.01 V output
        public static final double elevatorkP = 10; // A position error of 0.2 rotations results in 12 V output
        public static final double elevatorkI = 0; // No output for integrated error
        public static final double elevatorkD = 0;
        public static final double elevatorkG = 0.512;  
    }
    public static class ClimberConstants{
        public static final int kClimberID = 9;
        public static final double kClimberGearRatio = 375;
        public static final double climberkS = 0.2; // Add 0.25 V output to overcome static friction
        public static final double climberkV = 30; // A velocity target of 1 rps results in 0.12 V output
        public static final double climberkA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double climberkP = 100; // A position error of 0.2 rotations results in 12 V output
        public static final double climberkI = 0; // No output for integrated error
        public static final double climberkD = 10.897; 
        public static final double climberkG = 0.1; 

    }


    
}
