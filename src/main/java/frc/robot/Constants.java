package frc.robot;

public final class Constants {

    public static final double INCHES_PER_METER = 39.37;
    public static final double LENGTH_OF_BOT = 35 / INCHES_PER_METER;
    public static final double FIELD_OFFSET_FROM_NODE_TO_APRILTAG = 0.36;
    public static final double FIELD_OFFSET_FROM_SUBSTATION_TO_APRILTAG = -15 / INCHES_PER_METER;
    public static final double MID_SCORING_STANDOFF_DISTANCE = (25 - 4) / INCHES_PER_METER;
    public static final double BUMPER_THICKNESS = 3 / INCHES_PER_METER;

    // FRONT LEFT
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 1; 
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0; 
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 2; 
    public static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(6.46); //3.01

    // FRONT RIGHT
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 3; 
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 1;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 4;
    public static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(12.67); // 3.0775

    // BACK LEFT
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 5; 
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 2;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 6; 
    public static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(118.06); //2.9835

    // BACK RIGHT
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 7;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 3;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 8; 
    public static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-23.57); //3.0346

    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 42; // neo encoder ticks per revolution
    public static final double kWheelDiameterMeters = util.inchesToMeters(4.11);
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) * (1.0 / (60.0 / 15.0) / (20.0 / 24.0) / (40.0 / 16.0));

    // put into manual mode, manually read position and rotate wheel

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

    // new buttons - left
    public static final int CLAW_PNEUMATIC_BUTTON = 1;
    public static final int ARM_MANUAL_MOVEMENT_ACTIVATION_BUTTON = 2;
    public static final int NORMAL_BOTTOM_ARM_MOVEMENT = 3;
    public static final int NORMAL_MID_ARM_MOVEMENT = 4;
    public static final int ALIGN_TO_OBJECT_ON_FLOOR_BUTTON = 5;
    public static final int NORMAL_TOP_ARM_MOVEMENT = 6;
    public static final int NORMAL_MODE = 7;
    public static final int BALANCING_BUTTON = 8;
    public static final int HOLD_STILL_BUTTON = 9;
    public static final int RESET_DRIVE_BUTTON = 10;
    public static final int DEATH_CUBE_BUTTON = 11;
    public static final int DEATH_CONE_BUTTON = 12;

    // buttons - right
    public static final int ARM_PNEUMATIC_BUTTON = 1;
    public static final int SCORE_CENTER_BUTTON = 2;
    public static final int SUBSTATION_BUTTON = 3;
    public static final int PICKUP_FROM_FLOOR_BUTTON = 4;
    public static final int TRAVEL_BUTTON_ID = 5;
    public static final int HOME_ARM_BUTTON = 6;
    public static final int SCORE_TOP_LEFT = 7;
    public static final int SCORE_TOP_RIGHT = 8;
    public static final int SCORE_MID_LEFT = 9;
    public static final int SCORE_MID_RIGHT = 10;
    public static final int SCORE_BOTTOM_LEFT = 11;
    public static final int SCORE_BOTTOM_RIGHT = 12;

    

    // hat constants 
    public static final int HAT_JOYSTICK_TRIM_POSITION = RIGHT_JOYSTICK_CHANNEL;
    public static final int HAT_JOYSTICK_TRIM_ROTATION_ARM = LEFT_JOYSTICK_CHANNEL;
    public static final double HAT_POWER_MOVE = 0.05;
    public static final double HAT_POWER_ROTATE = 0.2;
    // Hat trim target speed is 15 degrees per second
    // One time step is 0.02 seconds
    // 0.3 degrees per time step is our target change when the hat is active
    public static final double HAT_POSE_TARGET_PER_TIME_STEP = -0.3; // negative is raising the arm
    public static final int HAT_POV_MOVE_LEFT = 270;
    public static final int HAT_POV_MOVE_RIGHT = 90;
    public static final int HAT_POV_MOVE_FORWARD = 0;
    public static final int HAT_POV_MOVE_BACK = 180;
    public static final int HAT_POV_ARM_UP = 0;
    public static final int HAT_POV_ARM_DOWN = 180;
    public static final int HAT_POV_ROTATE_LEFT = 270;
    public static final int HAT_POV_ROTATE_RIGHT = 90;

    // pneumatic channels
    public static final int PNEUMATIC_HUB_CANID = 15;
    public static final int CLOSE_CHANNEL = 1;
    public static final int OPEN_CHANNEL = 5;
    public static final int EXTEND_CHANNEL = 7;
    public static final int RETRACT_CHANNEL = 0;

    // arm angle positions
    public static final int HOME_ARM_ANGLE = 40; 
    public static final int TRAVELING_ARM_ANGLE_NOT_BLOCKING_CHASSIS_CAM = 295;
    public static final int TOP_SCORING_ANGLE = 207;
    public static final int TOP_TRANSITION_ANGLE = 210;
    public static final int MID_SCORING_ANGLE = 233;
    public static final int BOTTOM_SCORING_ANGLE = 326;
    public static final int SUBSTATION_ANGLE = 279;
    public static final boolean HOME_EXTEND = false;
    public static final boolean TRAVELING_ARM_EXTEND = false;
    public static final boolean TOP_SCORING_EXTEND = true;
    public static final boolean MID_SCORING_EXTEND = true;
    public static final boolean BOTTOM_SCORING_EXTEND = true;
    public static final boolean PICKING_UP_EXTEND = true;
    public static final boolean SUBSTATION_EXTEND = false;

    // pickup off floor
    public static final boolean ARM_EXTEND_DEATH_BUTTON_START = false; 
    public static final int ARM_ANGLE_DEATH_BUTTON_START = 302;
    public static final boolean ARM_EXTEND_PICKUP_FLOOR = true;
    public static final int ARM_ANGLE_PICKUP_FLOOR = 337;
    
    // illegal arm regions
    public static final int Hplus = 271;
    public static final int Hminus = 217;
    public static final int Vplus = 208;
    public static final int Vminus = 148;
    
    // arm motor constants
    public static final int ARM_EXTENDED_ALPHA = 116;
    public static final int ARM_EXTENDED_LOWER_LIMIT = 50;
    public static final int ARM_EXTENDED_UPPER_LIMIT = 360;

    public static final int ARM_RETRACTED_ALPHA = 80;
    public static final int ARM_RETRACTED_LOWER_LIMIT = 25;
    public static final int ARM_RETRACTED_UPPER_LIMIT = 335;

    public static final double FB_UPPER_LIMIT_CLOSED = 0.2;
    public static final double FB_LOWER_LIMIT_CLOSED = -0.2;
    public static final double FB_UPPER_LIMIT_OPEN = 0.35;
    public static final double FB_LOWER_LIMIT_OPEN = -0.35;

    public static final double ARM_MOTOR_FF_GAIN = -0.15;
    public static final double ARM_ENCODER_OFFSET = -349;

    // absolute enconder for top arm id
    public static final int ARM_ENCODER_ID = 7;

    // arm driving motor id
    public static final int TALON_CHANNEL = 21;

    // VISION CONSTANTS
    public static double OBJECT_DETECTION_LATENCY = 0.217; // seconds

    // how far away you quit driving w/ vision
    // TODO: figure out these distances
    public static final int TARGET_TRIGGER_DISTANCE_CONE = 10;       // cone is skinny
    public static final int TARGET_TRIGGER_DISTANCE_CUBE = 10;       // cube is not as skinny
    public static final int TARGET_TRIGGER_DISTANCE_ANY = 10;        // cube is not as skinny
    public static final int TARGET_TRIGGER_DISTANCE_APRIL_TAG = 0;   // april tag approach distance for score.  Can't see tag if closer


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

    public static final double offsetFromAprilTagToConeNode = 22 / INCHES_PER_METER;
    public static final double offsetFromAprilTagToCenter = 0;
    public static final double offsetFromAprilTagToSlider = 34 / INCHES_PER_METER;

    public static final String TARGET_OBJECT_LABEL_CONE = "cone";
    public static final String TARGET_OBJECT_LABEL_CUBE = "cube";
    public static final String TARGET_OBJECT_LABEL_ANY = "any";
    public static final String TARGET_OBJECT_LABEL_APRIL_TAG = "tag16h5";
//Robot Gripper Motor States
    public enum GRIPPER_MOTOR_STATE { Off, 
                            IdleOn, 
                            IntakeCube, 
                            IntakeCone,
                            OuttakeCube,
                            OuttakeConeDrop,
                            OuttakeConeEject
                          };
                        }