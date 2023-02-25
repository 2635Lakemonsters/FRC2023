// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ARM_TRANSITION;
import frc.robot.commands.ArmMovementCommand;
import frc.robot.commands.ArmPneumaticCommand;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.ClawPneumaticCommand;
import frc.robot.commands.MoveArmToPoseCommand;
import frc.robot.commands.ResetSwerveGyroCommand;
import frc.robot.commands.SetTargetPoseCommand;
import frc.robot.commands.SwerveAutoBalanceCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveNoMoveCommand;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import frc.robot.subsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class RobotContainer extends TimedRobot {
  // Joysticks
  public final static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public final static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

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
  private final ResetSwerveGyroCommand m_resetSwerveGyroCommand = new ResetSwerveGyroCommand(m_drivetrainSubsystem);
  private final SwerveDriveCommand m_swerveDriveCommand = new SwerveDriveCommand(m_drivetrainSubsystem);
  private final SwerveAutoBalanceCommand m_swerveDriveBalanceCommand = new SwerveAutoBalanceCommand(m_drivetrainSubsystem);
  private final SwerveNoMoveCommand m_swerveNoMoveCommand = new SwerveNoMoveCommand(m_drivetrainSubsystem);
  private final ClawPneumaticCommand m_clawOpenCommand = new ClawPneumaticCommand(m_clawPneumaticSubsystem, true);
  private final ClawPneumaticCommand m_clawCloseCommand = new ClawPneumaticCommand(m_clawPneumaticSubsystem, false);
  private final ArmPneumaticCommand m_armExtendCommand = new ArmPneumaticCommand(m_armPneumaticSubsystem, true);
  private final ArmPneumaticCommand m_armRetractCommand = new ArmPneumaticCommand(m_armPneumaticSubsystem, false);
  private final AutonomousCommands m_autonomousCommands = new AutonomousCommands();
  private final ArmMovementCommand m_armMovementCommand = new ArmMovementCommand(m_armMotorSubsystem, 90);


  public class Pose {
    public Pose() {
      this.targetExtend = false;
      this.targetTheta = Constants.ARM_RETRACTED_LOWER_LIMIT;
    }
    
    /**
    @param targetExtend whether arm pneumatics are extended (true) or not (false)
    @param targetTheta angle of upper arm relative to lower arm (NOT floor)
    **/
    public Pose(boolean targetExtend, int targetTheta) {
      this.targetExtend = targetExtend;
      this.targetTheta = targetTheta;
    }
    public boolean targetExtend;
    public int targetTheta;
    
  };

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
    // Create button
    // Trigger recalibrateButton = new JoystickButton(rightJoystick, Constants.CALIBRATE_BUTTON);
    // Trigger nonBalancingButton = new JoystickButton(rightJoystick, Constants.NORMAL_MODE);
    // Trigger balancingButton = new JoystickButton(rightJoystick, Constants.BALANCING_BUTTON);
    // Trigger stationaryButton = new JoystickButton(rightJoystick, Constants.HOLD_STILL_BUTTON);
    Trigger clawPneumaticButton = new JoystickButton(leftJoystick, Constants.CLAW_PNEUMATIC_BUTTON);
    Trigger armPneumaticButton = new JoystickButton(rightJoystick, Constants.ARM_PNEUMATIC_BUTTON);
    Trigger scoreTopRight = new JoystickButton(rightJoystick, Constants.SCORE_TOP_RIGHT);
    Trigger scoreMidRight = new JoystickButton(rightJoystick, Constants.SCORE_MID_RIGHT);
    Trigger scoreBottomRight = new JoystickButton(rightJoystick, Constants.SCORE_BOTTOM_RIGHT);

    Trigger armMovement = new JoystickButton(leftJoystick, 5);
    Trigger pickUpFloor = new JoystickButton(rightJoystick, 3);

    Trigger topLeftButton = new JoystickButton(rightJoystick, Constants.SCORE_TOP_LEFT);
    Trigger midLeftButton = new JoystickButton(rightJoystick, Constants.SCORE_MID_LEFT);
    Trigger bottomLeftButton = new JoystickButton(rightJoystick, Constants.SCORE_BOTTOM_LEFT);

    Trigger centerButton = new JoystickButton(rightJoystick, Constants.SCORE_CENTER_BUTTON);

    Trigger scoreTopLeft = centerButton.negate().and(topLeftButton);
    Trigger scoreTopCenter = centerButton.and(topLeftButton);
    Trigger scoreMidLeft = centerButton.negate().and(midLeftButton);
    Trigger scoreMidCenter = centerButton.and(midLeftButton);
    Trigger scoreBottomLeft = centerButton.negate().and(bottomLeftButton);
    Trigger scoreBottomCenter = centerButton.and(bottomLeftButton);

    // Set commmands to button
    // recalibrateButton.onTrue(m_resetSwerveGyroCommand);
    // balancingButton.onTrue(m_swerveDriveBalanceCommand);
    // nonBalancingButton.onTrue(m_swerveDriveCommand);
    // stationaryButton.onTrue(m_swerveNoMoveCommand);
    // clawPneumaticButton.onTrue(m_clawPneumaticCommand);
    // armPneumaticButton.onTrue(m_armPneumaticCommand);
    // armMovement.onTrue(m_armMovementCommand);
    // pickUpFloor.onTrue(new PickingUpArmMovementCommand(m_armPneumaticSubsystem, m_armMotorSubsystem).unless(() -> util.getArmState() != ARM_STATE.Bplus));

    // TODO:
    // 1: Do all the button bindings with arm movements
    // 2: Test Full Score movement portion
    // 3: Get target movement and rotation from April Tags, integrate into movement.
    // 4: Integrate both to parallel execution
    //SetTargetPoseCommand m_xxx = new SetTargetPoseCommand(new Pose(true, Constants.TOP_SCORING_ANGLE));

    // scoreTopLeft.onTrue(new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE)).andThen(new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)));
    
    // make into sequential command group
    scoreTopLeft.onTrue(  new SequentialCommandGroup(
                          new PrintCommand("Before STPC"),
                          new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE)), 
                          new PrintCommand("Before MATPC"),
                          new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                          new PrintCommand("After STPC")
                        ));
    
    scoreMidLeft.onTrue(  new SequentialCommandGroup( 
                          new SetTargetPoseCommand(new Pose(Constants.MID_SCORING_EXTEND, Constants.MID_SCORING_ANGLE)),
                          new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                        ));
    
    scoreBottomLeft.onTrue( new SequentialCommandGroup( 
                            new SetTargetPoseCommand(new Pose(Constants.BOTTOM_SCORING_EXTEND, Constants.BOTTOM_SCORING_ANGLE)),
                            new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                          ));

    scoreTopRight.onTrue( new SequentialCommandGroup(
                          new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE)),
                          new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                        ));

    scoreMidRight.onTrue( new SequentialCommandGroup(
                          new SetTargetPoseCommand(new Pose(Constants.MID_SCORING_EXTEND, Constants.MID_SCORING_ANGLE)),
                          new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                        ));

    scoreBottomRight.onTrue(  new SequentialCommandGroup(
                              new SetTargetPoseCommand(new Pose(Constants.BOTTOM_SCORING_EXTEND, Constants.BOTTOM_SCORING_ANGLE)),
                              new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                            ));

    scoreTopCenter.onTrue(  new SequentialCommandGroup(
                            new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE)),
                            new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                          ));

    scoreMidCenter.onTrue(  new SequentialCommandGroup(
                            new SetTargetPoseCommand(new Pose(Constants.MID_SCORING_EXTEND, Constants.MID_SCORING_ANGLE)),
                            new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                          ));

    scoreBottomCenter.onTrue( new SequentialCommandGroup(
                              new SetTargetPoseCommand(new Pose(Constants.BOTTOM_SCORING_EXTEND, Constants.BOTTOM_SCORING_ANGLE)),
                              new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                            ));

    // does two commands in parallel: 1) moving arm and 2) either moves to left or mid scoring position depending on state of button
    // raise arm + robot translation 
    // scoreTopLeft.onTrue(
    //   new ParallelCommandGroup( // TODO might want these to be sequential? bc you could hit the wall if you approach too close before you raise arm.  
    //     new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE))
    //     .andThen(m_moveArmCommand)),
    //     new ConditionalCommand(moveToMidScoringPosition, moveToLeftScoringPosition, leftJoystick.button(2)
    //             ));




    // armMovement.onTrue(m_armExtendCommand.unless(()->util.getArmState()!= ARM_STATE.Fplus));

    // armMovement.onTrue(new ConditionalCommand(m_armMovementCommand, m_armExtendCommand, ()->util.getArmState()!= ARM_STATE.Fplus));
  }

    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SendableChooser<Command> getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    m_autoChooser.setDefaultOption("Do Nothing", m_autonomousCommands.DoNothing());//establish default auto option

    // create other options in SmartDashBoard
    m_autoChooser.addOption("Out", m_autonomousCommands.OutPath(m_drivetrainSubsystem));
    m_autoChooser.addOption("Rotation 180", m_autonomousCommands.RotatePath(m_drivetrainSubsystem));
    m_autoChooser.addOption("Bottom score twice", m_autonomousCommands.BottomScoreTwice(m_drivetrainSubsystem));
    m_autoChooser.addOption("Mid score twice", m_autonomousCommands.MidScoreTwice(m_drivetrainSubsystem));
    m_autoChooser.addOption("Top score twice", m_autonomousCommands.TopScoreTwice(m_drivetrainSubsystem));
    m_autoChooser.addOption("Score bottom grab", m_autonomousCommands.ScoreBottomGrab(m_drivetrainSubsystem));
    m_autoChooser.addOption("Score mid grab", m_autonomousCommands.ScoreMidGrab(m_drivetrainSubsystem));
    m_autoChooser.addOption("Score top grab", m_autonomousCommands.ScoreTopGrab(m_drivetrainSubsystem));
    
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    return m_autoChooser; 
  }
}
