// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ARM_TRANSITION;
import frc.robot.commands.AlignGripperToObjectCommand;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.ClawPneumaticCommand;
import frc.robot.commands.ManualArmMotorCommand;
import frc.robot.commands.MoveArmToPoseCommand;
import frc.robot.commands.MoveToScore;
import frc.robot.commands.ResetSwerveGyroCommand;
import frc.robot.commands.SetTargetPoseCommand;
import frc.robot.commands.SwerveAutoBalanceCommand;
import frc.robot.commands.SwerveAutoBalanceCommandFEEDBACK;
import frc.robot.commands.SwerveDriveBalanceJerky;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveNoMoveCommand;
import frc.robot.commands.ToggleClawPneumaticsCommand;
import frc.robot.commands.VisionDriveClosedLoopCommand;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import frc.robot.subsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class RobotContainer extends TimedRobot {
  public static PathPlannerTrajectory traj;
  // Joysticks
  public static final Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public static final Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

  // Arm Encoder
  public static final AnalogInput encoder = new AnalogInput(Constants.ARM_ENCODER_ID);
  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  public static final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  public static final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  public static final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Subsystems
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static final ArmPneumaticSubsystem m_armPneumaticSubsystem = new ArmPneumaticSubsystem();
  public static final ClawPneumaticSubsystem m_clawPneumaticSubsystem = new ClawPneumaticSubsystem();
  public static final ObjectTrackerSubsystem m_objectTrackerSubsystemGripper = new ObjectTrackerSubsystem("Gripper");
  public static final ObjectTrackerSubsystem m_objectTrackerSubsystemChassis = new ObjectTrackerSubsystem("Chassis");
  public static final ArmMotorSubsystem m_armMotorSubsystem = new ArmMotorSubsystem();

  // Commands
  public final ResetSwerveGyroCommand m_resetSwerveGyroCommand = new ResetSwerveGyroCommand(m_drivetrainSubsystem);
  private final SwerveDriveCommand m_swerveDriveCommand = new SwerveDriveCommand(m_drivetrainSubsystem);
  private final SwerveAutoBalanceCommand m_swerveDriveBalanceCommand = new SwerveAutoBalanceCommand(m_drivetrainSubsystem);
  private final SwerveAutoBalanceCommandFEEDBACK m_swerveAutoBalanceCommandFEEDBACK = new SwerveAutoBalanceCommandFEEDBACK(m_drivetrainSubsystem);
  private final SwerveNoMoveCommand m_swerveNoMoveCommand = new SwerveNoMoveCommand(m_drivetrainSubsystem);
  private final AutonomousCommands m_autonomousCommands = new AutonomousCommands(m_drivetrainSubsystem, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, m_objectTrackerSubsystemGripper, m_objectTrackerSubsystemChassis);
  private final VisionDriveClosedLoopCommand m_visionDriveClosedLoopCommandCONE = new VisionDriveClosedLoopCommand(Constants.TARGET_OBJECT_LABEL_CONE, m_drivetrainSubsystem, m_objectTrackerSubsystemChassis);
  private final VisionDriveClosedLoopCommand m_visionDriveClosedLoopCommandCUBE = new VisionDriveClosedLoopCommand(Constants.TARGET_OBJECT_LABEL_CUBE, m_drivetrainSubsystem, m_objectTrackerSubsystemChassis);
  private final ManualArmMotorCommand m_manualArmMotorCommand = new ManualArmMotorCommand(m_armMotorSubsystem);

  // TODO: This probably isn't the best way to do this.  However, it'll do
  // for now and allow re-thinking later.  Use the set/getTargetPose functions
  // to insulate consumers from changes in implementation.
  
  private static Pose m_targetPose;

  public static Pose getTargetPose() {
    return m_targetPose;
  }

  public static void setTargetPose(Pose pose) {
    m_targetPose = pose;
  }

  public static void setTargetPose(boolean targetExtend, int targetTheta) {
    m_targetPose.targetExtend = targetExtend;
    m_targetPose.targetTheta = targetTheta;
}

  public ARM_TRANSITION select() {
    Pose pose = getTargetPose();
    return util.getTransition(pose.targetExtend, pose.targetTheta);
  }

  public interface Poser {
    public Pose execute();
  }

  public static Poser m_getPose = () -> m_targetPose;
  
  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(new SwerveDriveCommand(m_drivetrainSubsystem));
    configureBindings();
  }

  private void configureBindings() {
    // left joystick button bindings
    Trigger clawPneumaticButton = new JoystickButton(leftJoystick, Constants.CLAW_PNEUMATIC_BUTTON);
    Trigger manualArmMovement = new JoystickButton(leftJoystick, Constants.MANUAL_ARM_MOVEMENT_BUTTON);
    Trigger alignToObjectOnFloor = new JoystickButton(leftJoystick, Constants.ALIGN_TO_OBJECT_ON_FLOOR_BUTTON);
    Trigger homeArmButton = new JoystickButton(leftJoystick, Constants.HOME_ARM_BUTTON);
    Trigger nonBalancingButton = new JoystickButton(leftJoystick, Constants.NORMAL_MODE);
    Trigger balancingButton = new JoystickButton(leftJoystick, Constants.BALANCING_BUTTON);
    Trigger stationaryButton = new JoystickButton(leftJoystick, Constants.HOLD_STILL_BUTTON);
    Trigger resetDriveOrientation = new JoystickButton(leftJoystick, Constants.RESET_DRIVE_BUTTON);
    Trigger deathDriveCUBE = new JoystickButton(leftJoystick, Constants.DEATH_CUBE_BUTTON);
    Trigger deathDriveCONE = new JoystickButton(leftJoystick, Constants.DEATH_CONE_BUTTON);
    
    // right joystick button bindings
    Trigger armPneumaticButton = new JoystickButton(rightJoystick, Constants.ARM_PNEUMATIC_BUTTON);
    Trigger centerButton = new JoystickButton(rightJoystick, Constants.SCORE_CENTER_BUTTON);
    Trigger pickUpGamePieceSliderLeft = new JoystickButton(rightJoystick, Constants.LEFT_SLIDER_BUTTON);
    Trigger pickUpGamePieceSliderRight = new JoystickButton(rightJoystick, Constants.RIGHT_SLIDER_BUTTON);
    Trigger pickUpFromFloor = new JoystickButton(rightJoystick, Constants.PICKUP_FROM_FLOOR_BUTTON);
    Trigger topLeftButton = new JoystickButton(rightJoystick, Constants.SCORE_TOP_LEFT);
    Trigger scoreTopRight = new JoystickButton(rightJoystick, Constants.SCORE_TOP_RIGHT);
    Trigger midLeftButton = new JoystickButton(rightJoystick, Constants.SCORE_MID_LEFT);
    Trigger scoreMidRight = new JoystickButton(rightJoystick, Constants.SCORE_MID_RIGHT);
    Trigger bottomLeftButton = new JoystickButton(rightJoystick, Constants.SCORE_BOTTOM_LEFT);
    Trigger scoreBottomRight = new JoystickButton(rightJoystick, Constants.SCORE_BOTTOM_RIGHT);

    Trigger scoreTopLeft = centerButton.negate().and(topLeftButton);
    Trigger scoreTopCenter = centerButton.and(topLeftButton);
    Trigger scoreMidLeft = centerButton.negate().and(midLeftButton);
    Trigger scoreMidCenter = centerButton.and(midLeftButton);
    Trigger scoreBottomLeft = centerButton.negate().and(bottomLeftButton);
    Trigger scoreBottomCenter = centerButton.and(bottomLeftButton);


    PPSwerveControllerCommand.setLoggingCallbacks(PPLogging::logActiveTrajectory, PPLogging::logTargetPose, PPLogging::logSetpoint, PPLogging::logError);

    traj = PathPlanner.generatePath(
      new PathConstraints(2, 0.5), 
      new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
        new PathPoint(new Translation2d(-2.022, 0.78), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0))//6 * Math.PI / 3.09)) // position, heading(direction of travel)
        // new PathPoint(new Translation2d(0, 1), Rotation2d.fromRadians(0) // position, heading(direction of travel)
    );
    
    /** Have to do the following lines every single time you use the trajectory follower
     * 1. zero odometry
     * 2. followPath() --> IGNORES THE JOYSTICKS / DRIVE ELSE THEY WILL FIGHT BAD
     * 3. Run path follower
     * 4. Re-enable joysticks
     */
    // Command dsc = m_drivetrainSubsystem.followTrajectoryCommand(traj, true);
    // driveStraightButton.onTrue(new SequentialCommandGroup(
    //   new InstantCommand(()->m_drivetrainSubsystem.zeroOdometry()),
    //   new InstantCommand(()->m_drivetrainSubsystem.followPath()),
    //   dsc.andThen(() -> m_drivetrainSubsystem.drive(0, 0, 0, false)),
    //   new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
    // ));


    alignToObjectOnFloor.onTrue(new SequentialCommandGroup(
      new InstantCommand(()->m_drivetrainSubsystem.followPath()),
      new VisionDriveClosedLoopCommand(Constants.TARGET_OBJECT_LABEL_CUBE, true, m_drivetrainSubsystem, m_objectTrackerSubsystemChassis),
      new PrintCommand("Reached the cube"),
      new AlignGripperToObjectCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemGripper, m_armPneumaticSubsystem, m_clawPneumaticSubsystem),
      new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
      ));


    clawPneumaticButton.onTrue(new ToggleClawPneumaticsCommand(m_clawPneumaticSubsystem));

    manualArmMovement.whileTrue(m_manualArmMotorCommand);

    armPneumaticButton.toggleOnTrue(Commands.startEnd(m_armPneumaticSubsystem::armExtend,
                                      m_armPneumaticSubsystem::armRetract,
                                      m_armPneumaticSubsystem
                                    ));
    
    // TODO 3/2 uncomment out the following align commands and fix 
    pickUpFromFloor.onTrue(new SequentialCommandGroup(new SetTargetPoseCommand(new Pose(Constants.ARM_EXTEND_PICKUP_FLOOR, Constants.ARM_ANGLE_PICKUP_FLOOR)), 
                          new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                          // new AlignGripperToObjectCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemGripper, m_armPneumaticSubsystem, m_clawPneumaticSubsystem), 
                          new PrintCommand("after align gripper to object")
                          // m_clawCloseCommand,
                          // new SetTargetPoseCommand(new Pose(Constants.ARM_EXTEND_DEATH_BUTTON_START, Constants.ARM_ANGLE_DEATH_BUTTON_START)), 
                          // new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                          ));

    // comment out one of the balancing buttons based on which balance command we're using 
    // balancingButton.onTrue(new InstantCommand(() -> m_swerveDriveCommand.enableBalance(true)));

    // ENGAGING BALANCE
    // balancingButton.onTrue(m_swerveDriveBalanceCommand);
    balancingButton.onTrue( new SequentialCommandGroup(
                            new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                            m_swerveAutoBalanceCommandFEEDBACK
                          ));

    nonBalancingButton.onTrue(m_swerveDriveCommand);
    stationaryButton.onTrue(m_swerveNoMoveCommand);

    // TODO:
    // 1: Do all the button bindings with arm movements
    // 2: Test Full Score movement portion
    // 3: Get target movement and rotation from April Tags, integrate into movement.
    // 4: Integrate both to parallel execution
    //SetTargetPoseCommand m_xxx = new SetTargetPoseCommand(new Pose(true, Constants.TOP_SCORING_ANGLE));

    // scoreTopLeft.onTrue(new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE)).andThen(new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)));
    
    // make into sequential command group
    resetDriveOrientation.onTrue(m_resetSwerveGyroCommand);

    scoreTopLeft.onTrue(  new SequentialCommandGroup(
                          new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE)), 
                          new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                          new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                          new PrintCommand("About to MoveToScore"),
                          new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, -Constants.offsetFromAprilTagToConeNode, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG),
                          new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
                        ));
    
    scoreMidLeft.onTrue(  new SequentialCommandGroup( 
                          new SetTargetPoseCommand(new Pose(Constants.MID_SCORING_EXTEND, Constants.MID_SCORING_ANGLE)),
                          new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                          new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                          new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, -Constants.offsetFromAprilTagToConeNode, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG + Constants.MID_SCORING_STANDOFF_DISTANCE),
                          new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
                        ));
    
    scoreBottomLeft.onTrue( new SequentialCommandGroup( 
                            new SetTargetPoseCommand(new Pose(Constants.BOTTOM_SCORING_EXTEND, Constants.BOTTOM_SCORING_ANGLE)),
                            new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                            new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                            new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, -Constants.offsetFromAprilTagToConeNode, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG),
                            new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
                          ));

    scoreTopRight.onTrue( new SequentialCommandGroup(
                          new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE)),
                          new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                          new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                          new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.offsetFromAprilTagToConeNode, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG),
                          new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
                        ));

    scoreMidRight.onTrue( new SequentialCommandGroup(
                          new SetTargetPoseCommand(new Pose(Constants.MID_SCORING_EXTEND, Constants.MID_SCORING_ANGLE)),
                          new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                          new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                          new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.offsetFromAprilTagToConeNode, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG + Constants.MID_SCORING_STANDOFF_DISTANCE),
                          new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
                        ));

    scoreBottomRight.onTrue(  new SequentialCommandGroup(
                              new SetTargetPoseCommand(new Pose(Constants.BOTTOM_SCORING_EXTEND, Constants.BOTTOM_SCORING_ANGLE)),
                              new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                              new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                              new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.offsetFromAprilTagToConeNode, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG),
                              new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
                            ));

    scoreTopCenter.onTrue(  new SequentialCommandGroup(
                            new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE)),
                            new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                            new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                            new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.offsetFromAprilTagToCenter, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG),
                            new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
                          ));

    scoreMidCenter.onTrue(  new SequentialCommandGroup(
                            new SetTargetPoseCommand(new Pose(Constants.MID_SCORING_EXTEND, Constants.MID_SCORING_ANGLE)),
                            new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                            new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                            new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.offsetFromAprilTagToCenter, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG + Constants.MID_SCORING_STANDOFF_DISTANCE),
                            new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
                          ));

    scoreBottomCenter.onTrue( new SequentialCommandGroup(
                              new SetTargetPoseCommand(new Pose(Constants.BOTTOM_SCORING_EXTEND, Constants.BOTTOM_SCORING_ANGLE)),
                              new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                              new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                            new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.offsetFromAprilTagToCenter, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG),
                            new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
                            ));

    pickUpGamePieceSliderLeft.onTrue( new SequentialCommandGroup(
                            new ClawPneumaticCommand(m_clawPneumaticSubsystem, true),
                            new SetTargetPoseCommand(new Pose(Constants.SUBSTATION_EXTEND, Constants.SUBSTATION_ANGLE)),
                            new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                            new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                            new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, -Constants.offsetFromAprilTagToSlider, Constants.FIELD_OFFSET_FROM_SUBSTATION_TO_APRILTAG),
                            new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
                            ));

    pickUpGamePieceSliderRight.onTrue( new SequentialCommandGroup(
                            new ClawPneumaticCommand(m_clawPneumaticSubsystem, true),
                            new SetTargetPoseCommand(new Pose(Constants.SUBSTATION_EXTEND, Constants.SUBSTATION_ANGLE)),
                            new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                            new InstantCommand(()->m_drivetrainSubsystem.followPath()),
                            new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.offsetFromAprilTagToSlider, Constants.FIELD_OFFSET_FROM_SUBSTATION_TO_APRILTAG),
                            new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
                            ));
    
    deathDriveCONE.whileTrue( new ParallelCommandGroup(
                            m_visionDriveClosedLoopCommandCONE,
                            new SequentialCommandGroup(
                              new SetTargetPoseCommand(new Pose(Constants.ARM_EXTEND_DEATH_BUTTON_START, Constants.ARM_ANGLE_DEATH_BUTTON_START)),
                              new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
    )));

    deathDriveCUBE.whileTrue( new ParallelCommandGroup(
                            m_visionDriveClosedLoopCommandCUBE, 
                            new SequentialCommandGroup(
                              new SetTargetPoseCommand(new Pose(Constants.ARM_EXTEND_DEATH_BUTTON_START, Constants.ARM_ANGLE_DEATH_BUTTON_START)),
                              new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                            )));

    homeArmButton.onTrue( new SequentialCommandGroup(
                            new SetTargetPoseCommand(new Pose(Constants.HOME_EXTEND, Constants.HOME_ARM_ANGLE)),
                            new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                          ));

    Trigger travelButton = new JoystickButton(rightJoystick, Constants.TRAVEL_BUTTON_ID);
    travelButton.onTrue(new SequentialCommandGroup(
      new SetTargetPoseCommand(new Pose(false, Constants.TRAVELING_ARM_ANGLE_NOT_BLOCKING_CHASSIS_CAM)),
      new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
    ));
  }

    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SendableChooser<Command> getAutonomousCommand() {
    SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    m_autoChooser.addOption("Do Nothing", m_autonomousCommands.DoNothing());//establish default auto option

    // create other options in SmartDashBoard
    m_autoChooser.addOption("Out", m_autonomousCommands.OutPath(m_drivetrainSubsystem));
    m_autoChooser.addOption("Rotation Testing", m_autonomousCommands.RotationTesting(m_drivetrainSubsystem));
    m_autoChooser.addOption("Swerve Auto Balance", m_autonomousCommands.SwerveAutoBalanceCommand(m_drivetrainSubsystem));
    // m_autoChooser.addOption("Left score twice engage", m_autonomousCommands.LeftScoreTwiceEngage(m_drivetrainSubsystem));
    // m_autoChooser.addOption("Mid score twice engage", m_autonomousCommands.MidScoreTwiceEngage(m_drivetrainSubsystem));
    // m_autoChooser.addOption("Right score twice engage", m_autonomousCommands.RightScoreTwiceEngage(m_drivetrainSubsystem));
    // m_autoChooser.addOption("Right score engage", m_autonomousCommands.RightScoreEngage(m_drivetrainSubsystem));
    // m_autoChooser.addOption("Mid score engage", m_autonomousCommands.MidScoreEngage(m_drivetrainSubsystem));
    // m_autoChooser.addOption("Left score engage", m_autonomousCommands.LeftScoreEngage(m_drivetrainSubsystem));
    m_autoChooser.addOption("Circle test", m_autonomousCommands.CircleTest(m_drivetrainSubsystem));
    m_autoChooser.addOption("Drive Straight PP Traj WPI swerve controller", m_autonomousCommands.driveStraightPP(m_drivetrainSubsystem));
    m_autoChooser.addOption("Drive Straight Normal Traj WPI swerve controller", m_autonomousCommands.driveStraight(m_drivetrainSubsystem));
    m_autoChooser.addOption("PP score left out engage CONE", m_autonomousCommands.scoreHighOutEngageLeftCone());
    m_autoChooser.addOption("Score high drive out", m_autonomousCommands.scoreHighDriveOut());
    m_autoChooser.addOption("Near substation score high taxi out engage CONE", m_autonomousCommands.scoreHighOutScoreSustationSIDE());
    m_autoChooser.addOption("SwerveAutoBalanceCommandFEEDBACK", new SwerveAutoBalanceCommand(m_drivetrainSubsystem));

    m_autoChooser.addOption("SwerveAutoBalanceCommandJerkey", new SequentialCommandGroup(
        new InstantCommand(()->m_drivetrainSubsystem.followPath()), 
        new SwerveDriveBalanceJerky(m_drivetrainSubsystem), 
        new InstantCommand(()->m_drivetrainSubsystem.followJoystick())
    ));

    m_autoChooser.addOption("scoreHighMobilityGrabScoreHigh", m_autonomousCommands.scoreHighMobilityGrabScoreHigh());

    m_autoChooser.setDefaultOption("Score High", new SequentialCommandGroup(m_autonomousCommands.scoreHigh(),
    new SetTargetPoseCommand(new Pose(Constants.HOME_EXTEND, Constants.HOME_ARM_ANGLE)),
    new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)));
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    return m_autoChooser; 
  }

}

