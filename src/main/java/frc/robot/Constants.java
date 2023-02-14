// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // FRONT LEFT
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 1; 
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0; 
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 2; 
    public static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-10);

    // FRONT RIGHT
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 3; 
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 1;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 4;
    public static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(21);

    // BACK LEFT
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 5; 
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 2;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 6; 
    public static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(160);

    // BACK RIGHT
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 7;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 3;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 8; 
    public static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(87);

    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterMeters = util.inchesToMeters(4.0);
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) * (1.0 / (60.0 / 15.0) / (20.0 / 24.0) / (40.0 / 16.0));

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 0.5;

    public static final double kPModuleDriveController = 0;

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // joystick channels
    public static final int RIGHT_JOYSTICK_CHANNEL = 1;
    public static final int LEFT_JOYSTICK_CHANNEL = 0;

    // buttons
    public static final int CLAW_PNEUMATIC_BUTTON = 1;
    public static final int ARM_PNEUMATIC_BUTTON = 1;
    public static final int CALIBRATE_BUTTON = 7;
    public static final int NORMAL_MODE = 8;
    public static final int BALANCING_BUTTON = 9;
    public static final int HOLD_STILL_BUTTON = 10;

    public static final int TOP_LEFT_CONE = 5;
    public static final int MID_LEFT_CONE = 3;
    public static final int TOP_RIGHT_CONE = 6;
    public static final int MID_RIGHT_CONE = 4;
    public static final int TOP_CUBE = 0;
    public static final int MID_CUBE = 180;

    // pneumatic channels
    public static final int CLOSE_CHANNEL = 0;
    public static final int OPEN_CHANNEL = 1;
    public static final int EXTEND_CHANNEL = 6;
    public static final int RETRACT_CHANNEL = 7;

    // VISION CONSTANTS
    public static double OBJECT_DETECTION_LATENCY = 0.217; // seconds

}
