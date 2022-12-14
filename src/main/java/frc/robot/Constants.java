// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.swerve.SwerveModuleAttributes;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static double wheelRadius = 0.04;
    public static double maxWheelVelocityMetersPerSecond = 5.3;
    public static double maxTraverseVelocityMetersPerSecond = 5;
    public static double maxRotationRadsPerSecond = 2.5;
    public static double xModeMaxSpeed = 0.3;
    public static boolean squareMode = false;

    public static double driveKp = 8.0;
    public static double driveKi = 1.2;
    public static double driveKd = 0.0;
    public static double angleKp = 50;
    public static double angleKi = 0;
    public static double angleKd = 2;
    
    public static SwerveModuleAttributes frontLeftAttributes = new SwerveModuleAttributes(1, 5, 90);
    public static SwerveModuleAttributes frontRightAttributes = new SwerveModuleAttributes(2, 6, 0);
    public static SwerveModuleAttributes backLeftAttributes = new SwerveModuleAttributes(3, 7, 0);
    public static SwerveModuleAttributes backRightAttributes = new SwerveModuleAttributes(4, 8, 0);
}

