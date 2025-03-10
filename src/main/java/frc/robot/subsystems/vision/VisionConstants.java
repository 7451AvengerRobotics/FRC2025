// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;


public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String limelight1Camera = "LimelightLeft";
  public static String limelight2Camera = "Limelight";
  public static String camera0Name = "FrontLeft";
  public static String camera1Name = "FrontRight";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d limelight3Transform3d =
    new Transform3d(Units.inchesToMeters(-11.65), Units.inchesToMeters(5.516), Units.inchesToMeters(5.887), 
    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-26), Units.degreesToRadians(170)));
  public static Transform3d limelight2Transform3d =
    new Transform3d(Units.inchesToMeters(-11.65), Units.inchesToMeters(-12.48), Units.inchesToMeters(8.371), 
    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-26), Units.degreesToRadians(190)));
  public static Transform3d frontRightTransform3d =
    new Transform3d(Units.inchesToMeters(11.024), Units.inchesToMeters(-12.48), Units.inchesToMeters(8.371), 
    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-28.125), Units.degreesToRadians(-30)));
  public static Transform3d frontLeftTransform3d =
    new Transform3d(Units.inchesToMeters(11.024), Units.inchesToMeters(12.48), Units.inchesToMeters(8.371), 
    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-28.125), Units.degreesToRadians(30)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.05; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1.0
      };
}