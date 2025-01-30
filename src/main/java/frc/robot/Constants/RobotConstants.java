package frc.robot.Constants;

public class RobotConstants {

    public static class IndexConstants{
        public static final int kIndexID = 23;
    }
    public static class IntakeConstants{
        public static final int kIntakePivotID = 24;
        public static final int kIntakeID = 25;
        public static final double kIntakeGearRatio = 125;
        public static final double intakePivotkS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double intakePivotkV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double intakePivotkA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double intakePivotkP = 60; // A position error of 0.2 rotations results in 12 V output
        public static final double intakePivotkI = 0; // No output for integrated error
        public static final double intakePivotkD = 0.5; 

    }
    public static class ClawConstants{
        public static final int kClawPivotID = 26;
        public static final int kClawID = 27;
        public static final double kClawGearRatio = 125;
        public static final double clawPivotkS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double clawPivotkV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double clawPivotkA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double clawPivotkP = 60; // A position error of 0.2 rotations results in 12 V output
        public static final double clawPivotkI = 0; // No output for integrated error
        public static final double clawPivotkD = 0.5; 
    }
    
    public static class ElevatorConstants {
        public static final int kElevatorID = 28;
        public static final double kElevatorGearRatio = 125;
        public static final double elevatorkS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double elevatorkV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double elevatorkA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double elevatorkP = 60; // A position error of 0.2 rotations results in 12 V output
        public static final double elevatorkI = 0; // No output for integrated error
        public static final double elevatorkD = 0.5; 
    }


    
}
