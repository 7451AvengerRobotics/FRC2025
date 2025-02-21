package frc.robot.Constants;

public class RobotConstants {

    public static class IndexConstants{
        public static final int kIndexID = 40;
    }
    public static class IntakeConstants{
        public static final int kIntakePivotID = 20;
        public static final int kIntakeID = 41;
        public static final double kIntakeGearRatio = 312.5;
        public static final double intakePivotkS = 0.11518; // Add 0.25 V output to overcome static friction
        public static final double intakePivotkV = 34.976; // A velocity target of 1 rps results in 0.12 V output
        public static final double intakePivotkA = 0.35824; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double intakePivotkP = 35.515; // A position error of 0.2 rotations results in 12 V output
        public static final double intakePivotkI = 0; // No output for integrated error
        public static final double intakePivotkD = 10.897; 
        public static final double intakePivotkG = 0.091684; 

    }
    public static class ClawConstants{
        public static final int kClawPivotID = 33;
        public static final int kClawID = 27;
        public static final double kClawGearRatio = 125;
        public static final double clawPivotkS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double clawPivotkV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double clawPivotkA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double clawPivotkP = 60; // A position error of 0.2 rotations results in 12 V output
        public static final double clawPivotkI = 0; // No output for integrated error
        public static final double clawPivotkD = 0.5; 
        public static final double clawPivotkG = 0.5; 
    }
    
    public static class ElevatorConstants {
        public static final int kElevatorID = 35;
        public static final double kElevatorGearRatio = 20;
        public static final double elevatorkS = 0.22089; // Add 0.25 V output to overcome static friction
        public static final double elevatorkV = 2.3206; // A velocity target of 1 rps results in 0.12 V output
        public static final double elevatorkA = 0.052889; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double elevatorkP = 107; // A position error of 0.2 rotations results in 12 V output
        public static final double elevatorkI = 0; // No output for integrated error
        public static final double elevatorkD = 6;
        public static final double elevatorkG = 0.31061;  
    }


    
}
