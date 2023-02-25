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
    public static final int RIGHT_JOYSTICK_CHANNEL = 2;
    public static final int LEFT_JOYSTICK_CHANNEL = 0;

    // buttons
    public static final int CLAW_PNEUMATIC_BUTTON = 1;
    public static final int ARM_PNEUMATIC_BUTTON = 1;
    public static final int CALIBRATE_BUTTON = 7;
    public static final int NORMAL_MODE = 8;
    public static final int BALANCING_BUTTON = 9;
    public static final int HOLD_STILL_BUTTON = 10;

    public static final int SCORE_TOP_LEFT = 7;
    public static final int SCORE_MID_LEFT = 9;
    public static final int SCORE_BOTTOM_LEFT = 11;
    public static final int SCORE_TOP_RIGHT = 8;
    public static final int SCORE_MID_RIGHT = 10;
    public static final int SCORE_BOTTOM_RIGHT = 12;
    public static final int SCORE_CENTER_BUTTON = 2;

    public static final int LEFT_SLIDER_BUTTON = 3;
    public static final int RIGHT_SLIDER_BUTTON = 4;

    // pneumatic channels
    public static final int PNEUMATIC_HUB_CANID = 15;
    public static final int CLOSE_CHANNEL = 1;
    public static final int OPEN_CHANNEL = 5;
    public static final int EXTEND_CHANNEL = 7;
    public static final int RETRACT_CHANNEL = 0;

    // arm angle positions
    public static final int HOME_ARM_ANGLE = 331;
    public static final int TOP_SCORING_ANGLE = 202;
    public static final int MID_SCORING_ANGLE = 268;
    public static final int BOTTOM_SCORING_ANGLE = 326;
    public static final int PICKING_UP_ANGLE = 302;
    public static final int SUBSTATION_ANGLE = 268; // TODO: check this value
    public static final boolean HOME_EXTEND = false;
    public static final boolean TOP_SCORING_EXTEND = true;
    public static final boolean MID_SCORING_EXTEND = false;
    public static final boolean BOTTOM_SCORING_EXTEND = true;
    public static final boolean PICKING_UP_EXTEND = true;
    public static final boolean SUBSTATION_EXTEND = false; // TODO: check this value


    // illegal arm regions
    public static final int Hplus = 273;
    public static final int Hminus = 210;
    public static final int Vplus = 201;        // Calculated as 200.34
    public static final int Vminus = 156;       // Calculated as 156.66
    
    // arm motor constants
    public static final int ARM_EXTENDED_ALPHA = 116;
    public static final int ARM_EXTENDED_LOWER_LIMIT = 50;
    public static final int ARM_EXTENDED_UPPER_LIMIT = 360;

    public static final int ARM_RETRACTED_ALPHA = 80;
    public static final int ARM_RETRACTED_LOWER_LIMIT = 25;
    public static final int ARM_RETRACTED_UPPER_LIMIT = 335;

    public static final double FB_UPPER_LIMIT = 0.2;
    public static final double FB_LOWER_LIMIT = -0.2;

    public static final double ARM_MOTOR_FF_GAIN = -0.08;
    public static final double ARM_ENCODER_OFFSET = -349;

    // absolute enconder for top arm id
    public static final int ARM_ENCODER_ID = 7;

    // arm driving motor id
    public static final int TALON_CHANNEL = 21;

    // VISION CONSTANTS
    public static double OBJECT_DETECTION_LATENCY = 0.217; // seconds
    public static final double TARGET_TRIGGER_DISTANCE = 36; // how far away you quit driving w/ vision


    public enum ARM_STATE { Fplus, 
                            Fminus, 
                            Bplus, 
                            Bminus,
                            InvalidVert,
                            InvalidHorz,
                            Moving};

    public enum ARM_TRANSITION { FPlus2FPlus,
                                 FPlus2FMinus,
                                 FPlus2BPlus,
                                 FPlus2BMinus,
                                 FMinus2FPlus,
                                 FMinus2FMinus,
                                 FMinus2BPlus,
                                 FMinus2BMinus,
                                 BPlus2FPlus,
                                 BPlus2FMinus,
                                 BPlus2BPlus,
                                 BPlus2BMinus,
                                 BMinus2FPlus,
                                 BMinus2FMinus,
                                 BMinus2BPlus,
                                 BMinus2BMinus,
                                 Illegal};

    public enum SCORING_LOCATION { Left, Middle, Right };

    public static final int  APRIL_TAG_ID_RedRight = 1;
    public static final int  APRIL_TAG_ID_RedMiddle = 2;
    public static final int  APRIL_TAG_ID_RedLeft = 3;
    public static final int  APRIL_TAG_ID_BlueSubstation = 4;
    public static final int  APRIL_TAG_ID_RedSubstation = 5;
    public static final int  APRIL_TAG_ID_BlueRight = 6;
    public static final int  APRIL_TAG_ID_BlueMiddle = 7;
    public static final int  APRIL_TAG_ID_BlueLeft = 8;

    public static final double offsetFromAprilTagToConeNode = 0.5588;
    public static final double offsetFromAprilTagToCenter = 0;
    public static final double offsetFromAprilTagToSlider = 34*2.54/100;
}
