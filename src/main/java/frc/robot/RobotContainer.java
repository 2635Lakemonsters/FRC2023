// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.robot.commands.MoveArmToPoseCommand;
import frc.robot.commands.MoveToScore;
import frc.robot.commands.ResetSwerveGyroCommand;
import frc.robot.commands.SetTargetPoseCommand;
import frc.robot.commands.SwerveAutoBalanceCommandFEEDBACK;
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
  public static final ArmMotorSubsystem m_armMotorSubsystem = new ArmMotorSubsystem(m_clawPneumaticSubsystem);

  // Commands
  public final ResetSwerveGyroCommand m_resetSwerveGyroCommand = new ResetSwerveGyroCommand(m_drivetrainSubsystem);
  private final SwerveDriveCommand m_swerveDriveCommand = new SwerveDriveCommand(m_drivetrainSubsystem);
  private final SwerveAutoBalanceCommandFEEDBACK m_swerveAutoBalanceCommandFEEDBACK = new SwerveAutoBalanceCommandFEEDBACK(m_drivetrainSubsystem);
  private final SwerveNoMoveCommand m_swerveNoMoveCommand = new SwerveNoMoveCommand(m_drivetrainSubsystem);
  private final AutonomousCommands m_autonomousCommands = new AutonomousCommands(m_drivetrainSubsystem, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, m_objectTrackerSubsystemGripper, m_objectTrackerSubsystemChassis, m_getPose);
  private final VisionDriveClosedLoopCommand m_visionDriveClosedLoopCommandCONE = new VisionDriveClosedLoopCommand(Constants.TARGET_OBJECT_LABEL_CONE, true, m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, true);
  private final VisionDriveClosedLoopCommand m_visionDriveClosedLoopCommandCUBE = new VisionDriveClosedLoopCommand(Constants.TARGET_OBJECT_LABEL_CUBE, true, m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, true);

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
    Trigger nonBalancingButton = new JoystickButton(leftJoystick, Constants.NORMAL_MODE);
    Trigger normalBottomArmMovement = new JoystickButton(leftJoystick, Constants.NORMAL_BOTTOM_ARM_MOVEMENT);
    Trigger normalMidArmMovement = new JoystickButton(leftJoystick, Constants.NORMAL_MID_ARM_MOVEMENT);
    Trigger normalTopArmMovement = new JoystickButton(leftJoystick, Constants.NORMAL_TOP_ARM_MOVEMENT);
    Trigger stationaryButton = new JoystickButton(leftJoystick, Constants.HOLD_STILL_BUTTON);
    Trigger resetDriveOrientation = new JoystickButton(leftJoystick, Constants.RESET_DRIVE_BUTTON);
    // right joystick button bindings
    Trigger armPneumaticButton = new JoystickButton(rightJoystick, Constants.ARM_PNEUMATIC_BUTTON);
    Trigger substationPickup = new JoystickButton(rightJoystick, Constants.SUBSTATION_BUTTON);
    Trigger pickUpFromFloor = new JoystickButton(rightJoystick, Constants.PICKUP_FROM_FLOOR_BUTTON);
    Trigger travelButton = new JoystickButton(rightJoystick, Constants.TRAVEL_BUTTON_ID);
    Trigger homeArmButton = new JoystickButton(rightJoystick, Constants.HOME_ARM_BUTTON);

    PPSwerveControllerCommand.setLoggingCallbacks(PPLogging::logActiveTrajectory, PPLogging::logTargetPose, PPLogging::logSetpoint, PPLogging::logError);
  
    clawPneumaticButton.onTrue(new ToggleClawPneumaticsCommand(m_clawPneumaticSubsystem));

    armPneumaticButton.toggleOnTrue(Commands.startEnd(m_armPneumaticSubsystem::armExtend,
                                      m_armPneumaticSubsystem::armRetract,
                                      m_armPneumaticSubsystem
                                    ));
    
    pickUpFromFloor.onTrue(new SequentialCommandGroup(new SetTargetPoseCommand(new Pose(Constants.ARM_EXTEND_PICKUP_FLOOR, Constants.ARM_ANGLE_PICKUP_FLOOR)), 
                          new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose),
                          new PrintCommand("after align gripper to object")
                          ));

    nonBalancingButton.onTrue(m_swerveDriveCommand);
    stationaryButton.onTrue(m_swerveNoMoveCommand);
    resetDriveOrientation.onTrue(m_resetSwerveGyroCommand);

    normalTopArmMovement.onTrue(  new SequentialCommandGroup(
                                  new InstantCommand(()->m_drivetrainSubsystem.followJoystick()),
                                  new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE)), 
                                  new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                          ));

    normalMidArmMovement.onTrue(  new SequentialCommandGroup(
                                  new InstantCommand(()->m_drivetrainSubsystem.followJoystick()),
                                  new SetTargetPoseCommand(new Pose(Constants.MID_SCORING_EXTEND, Constants.MID_SCORING_ANGLE)),
                                  new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                          ));

    normalBottomArmMovement.onTrue( new SequentialCommandGroup( 
                                    new InstantCommand(()->m_drivetrainSubsystem.followJoystick()),
                                    new SetTargetPoseCommand(new Pose(Constants.BOTTOM_SCORING_EXTEND, Constants.BOTTOM_SCORING_ANGLE)),
                                    new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                          ));

    substationPickup.onTrue(new SequentialCommandGroup(
                            new ClawPneumaticCommand(m_clawPneumaticSubsystem, true),
                            new SetTargetPoseCommand(new Pose(Constants.SUBSTATION_EXTEND, Constants.SUBSTATION_ANGLE + 4)),
                            new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                            ));

    homeArmButton.onTrue( new SequentialCommandGroup(
                          new SetTargetPoseCommand(new Pose(Constants.HOME_EXTEND, Constants.HOME_ARM_ANGLE)),
                          new MoveArmToPoseCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)
                        ));

    travelButton.onTrue(new SequentialCommandGroup(
      new SetTargetPoseCommand(new Pose(false, Constants.TRAVELING_ARM_ANGLE_NOT_BLOCKING_CHASSIS_CAM + 40)),
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
    m_autoChooser.setDefaultOption("Score High", new SequentialCommandGroup(m_autonomousCommands.scoreHigh()));
    m_autoChooser.addOption("Score high drive out", m_autonomousCommands.scoreHighDriveOut());
    m_autoChooser.addOption("RIGHT!!scoreHighMobilityGrabScoreHigh", m_autonomousCommands.scoreHighMobilityGrabScoreHighRight());
    m_autoChooser.addOption("LEFT!!scoreHighMobilityGrabScoreHigh", m_autonomousCommands.scoreHighMobilityGrabScoreHighLeft());
    m_autoChooser.addOption("Trajectory Test", m_autonomousCommands.trajectoryTest());
    m_autoChooser.addOption("Trajectory Test Negative", m_autonomousCommands.trajectoryTestNegative());

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    return m_autoChooser; 
  }
}

