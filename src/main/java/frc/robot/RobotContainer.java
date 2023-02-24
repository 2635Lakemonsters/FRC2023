// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import org.ejml.ops.QuickSort_S32;

// import com.revrobotics.AnalogInput;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ARM_STATE;
import frc.robot.Constants.ARM_TRANSITION;
import frc.robot.commands.ArmMovementCommand;
import frc.robot.commands.ArmPneumaticCommand;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.ClawPneumaticCommand;
import frc.robot.commands.FullScoringCommand;
import frc.robot.commands.PickingUpArmMovementCommand;
import frc.robot.commands.ResetSwerveGyroCommand;
import frc.robot.commands.ReturnToDockCommand;
import frc.robot.commands.SetTargetPoseCommand;
import frc.robot.commands.SwerveAutoBalanceCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveNoMoveCommand;
import frc.robot.commands.Transitions.BMinus2BMinusCommand;
import frc.robot.commands.Transitions.BMinus2BPlusCommand;
import frc.robot.commands.Transitions.BMinus2FMinusCommand;
import frc.robot.commands.Transitions.BMinus2FPlusCommand;
import frc.robot.commands.Transitions.BPlus2BMinusCommand;
import frc.robot.commands.Transitions.BPlus2BPlusCommand;
import frc.robot.commands.Transitions.BPlus2FMinusCommand;
import frc.robot.commands.Transitions.BPlus2FPlusCommand;
import frc.robot.commands.Transitions.FMinus2BMinusCommand;
import frc.robot.commands.Transitions.FMinus2BPlusCommand;
import frc.robot.commands.Transitions.FMinus2FMinusCommand;
import frc.robot.commands.Transitions.FMinus2FPlusCommand;
import frc.robot.commands.Transitions.FPlus2BMinusCommand;
import frc.robot.commands.Transitions.FPlus2BPlusCommand;
import frc.robot.commands.Transitions.FPlus2FMinusCommand;
import frc.robot.commands.Transitions.FPlus2FPlusCommand;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import frc.robot.subsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class RobotContainer extends TimedRobot {
  // Joysticks
  public final static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public final static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

  // Pneumatic Control Module
  //public static final PneumaticHub m_pneumaticHub = new PneumaticHub(Constants.PNEUMATIC_HUB_CANID);

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
  private final FullScoringCommand m_autoScoreTopLeft = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.TOP_LEFT_CONE);
  private final FullScoringCommand m_autoScoreMidLeft = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.MID_LEFT_CONE);
  private final FullScoringCommand m_autoScoreTopRight = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.TOP_RIGHT_CONE);
  private final FullScoringCommand m_autoScoreMidRight = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.MID_RIGHT_CONE);
  private final FullScoringCommand m_autoScoreTopCube = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.TOP_CUBE);
  private final FullScoringCommand m_autoScoreMidCube = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.MID_CUBE);
  private final AutonomousCommands m_autonomousCommands = new AutonomousCommands();
  private final ReturnToDockCommand m_returnToDockCommand = new ReturnToDockCommand(m_armPneumaticSubsystem, m_armMotorSubsystem);
  private final ArmMovementCommand m_armMovementCommand = new ArmMovementCommand(m_armMotorSubsystem, 90);

  public class Pose {
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

  static Pose getTargetPose() {
    return m_targetPose;
  }

  public static void setTargetPose(Pose pose) {
    m_targetPose = pose;
  }

  private ARM_TRANSITION select() {
    Pose pose = getTargetPose();
    return util.getTransition(pose.targetExtend, pose.targetTheta);
  }

  public interface Poser {
    public Pose execute();
  }

  static Poser m_getPose = () -> m_targetPose;
  
  private final Command m_moveArmCommand = new SelectCommand(
    // Maps selector values to commands
    Map.ofEntries(
      Map.entry(ARM_TRANSITION.BMinus2BMinus, new BMinus2BMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.BMinus2BPlus, new BMinus2BPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.BMinus2FMinus, new BMinus2FMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.BMinus2FPlus, new BMinus2FPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.BPlus2BMinus, new BPlus2BMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.BPlus2BPlus, new BPlus2BPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.BPlus2FMinus, new BPlus2FMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.BPlus2FPlus, new BPlus2FPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.FMinus2BMinus, new FMinus2BMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.FMinus2BPlus, new FMinus2BPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.FMinus2FMinus, new FMinus2FMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.FMinus2FPlus, new FMinus2FPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.FPlus2BMinus, new FPlus2BMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.FPlus2BPlus, new FPlus2BPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.FPlus2FMinus, new FPlus2FMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
      Map.entry(ARM_TRANSITION.FPlus2FPlus, new FPlus2FPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose))),
    this::select);

  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(new SwerveDriveCommand(m_drivetrainSubsystem));
    configureBindings();
  }

  private void configureBindings() {
    // Create button
    Trigger recalibrateButton = new JoystickButton(rightJoystick, Constants.CALIBRATE_BUTTON);
    Trigger nonBalancingButton = new JoystickButton(rightJoystick, Constants.NORMAL_MODE);
    Trigger balancingButton = new JoystickButton(rightJoystick, Constants.BALANCING_BUTTON);
    Trigger stationaryButton = new JoystickButton(rightJoystick, Constants.HOLD_STILL_BUTTON);
    Trigger clawPneumaticButton = new JoystickButton(leftJoystick, Constants.CLAW_PNEUMATIC_BUTTON);
    Trigger armPneumaticButton = new JoystickButton(rightJoystick, Constants.ARM_PNEUMATIC_BUTTON);
    Trigger scoreConeTopLeft = new JoystickButton(rightJoystick, Constants.TOP_LEFT_CONE);
    Trigger scoreConeMidLeft = new JoystickButton(rightJoystick, Constants.MID_LEFT_CONE);
    Trigger scoreConeTopRight = new JoystickButton(rightJoystick, Constants.TOP_RIGHT_CONE);
    Trigger scoreConeMidRight = new JoystickButton(rightJoystick, Constants.MID_RIGHT_CONE);
    POVButton scoreCubeTop = new POVButton(rightJoystick, Constants.TOP_CUBE);
    POVButton scoreCubeMid = new POVButton(rightJoystick, Constants.MID_CUBE);
    Trigger returnToDock = new JoystickButton(leftJoystick, Constants.DOCKING_BUTTON_NUMBER);
    Trigger armMovement = new JoystickButton(leftJoystick, 5);
    Trigger pickUpFloor = new JoystickButton(rightJoystick, 3);

    // Set commmands to button
    recalibrateButton.onTrue(m_resetSwerveGyroCommand);
    balancingButton.onTrue(m_swerveDriveBalanceCommand);
    nonBalancingButton.onTrue(m_swerveDriveCommand);
    stationaryButton.onTrue(m_swerveNoMoveCommand);
    // clawPneumaticButton.onTrue(m_clawPneumaticCommand);
    // armPneumaticButton.onTrue(m_armPneumaticCommand);
    //scoreConeTopLeft.onTrue(m_autoScoreTopLeft);
    scoreConeMidLeft.onTrue(m_autoScoreMidLeft);
    scoreConeTopRight.onTrue(m_autoScoreTopRight);
    scoreConeMidRight.onTrue(m_autoScoreMidRight);
    scoreCubeTop.onTrue(m_autoScoreTopCube);
    scoreCubeMid.onTrue(m_autoScoreMidCube);
    returnToDock.onTrue(m_returnToDockCommand);
    armMovement.onTrue(m_armMovementCommand);
    pickUpFloor.onTrue(new PickingUpArmMovementCommand(m_armPneumaticSubsystem, m_armMotorSubsystem).unless(() -> util.getArmState() != ARM_STATE.Bplus));

    SetTargetPoseCommand m_xxx = new SetTargetPoseCommand(new Pose(true, Constants.TOP_SCORING_ANGLE));
    scoreConeTopLeft.onTrue(m_xxx.andThen(m_moveArmCommand));


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
