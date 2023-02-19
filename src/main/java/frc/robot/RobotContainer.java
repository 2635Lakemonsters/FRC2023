// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc.robot.commands.ArmFeedforwardCommand;
import frc.robot.commands.ArmPneumaticCommand;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.ClawPneumaticCommand;
import frc.robot.commands.FullScoringCommand;
import frc.robot.commands.PrintGetXCommand;
import frc.robot.commands.ResetSwerveGyroCommand;
import frc.robot.commands.ReturnToDockCommand;
import frc.robot.commands.SwerveAutoBalanceCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveNoMoveCommand;
import frc.robot.subsystems.ArmFeedforwardSubsystem;
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
  public static final PneumaticHub m_pneumaticHub = new PneumaticHub(Constants.PNEUMATIC_HUB_CANID);

  // Arm Encoder
  public static final AnalogEncoder encoder = new AnalogEncoder(Constants.ARM_ENCODER_ID);

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
  public static final ArmFeedforwardSubsystem m_armFeedforwardSubsystem = new ArmFeedforwardSubsystem(m_armMotorSubsystem, m_armPneumaticSubsystem);

  // Commands
  private final ResetSwerveGyroCommand m_resetSwerveGyroCommand = new ResetSwerveGyroCommand(m_drivetrainSubsystem);
  private final SwerveDriveCommand m_swerveDriveCommand = new SwerveDriveCommand(m_drivetrainSubsystem);
  private final SwerveAutoBalanceCommand m_swerveDriveBalanceCommand = new SwerveAutoBalanceCommand(m_drivetrainSubsystem);
  private final SwerveNoMoveCommand m_swerveNoMoveCommand = new SwerveNoMoveCommand(m_drivetrainSubsystem);
  private final ClawPneumaticCommand m_clawPneumaticCommand = new ClawPneumaticCommand(m_clawPneumaticSubsystem);
  private final ArmPneumaticCommand m_armPneumaticCommand = new ArmPneumaticCommand(m_armPneumaticSubsystem);
  private final FullScoringCommand m_autoScoreTopLeft = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.TOP_LEFT_CONE);
  private final FullScoringCommand m_autoScoreMidLeft = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.MID_LEFT_CONE);
  private final FullScoringCommand m_autoScoreTopRight = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.TOP_RIGHT_CONE);
  private final FullScoringCommand m_autoScoreMidRight = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.MID_RIGHT_CONE);
  private final FullScoringCommand m_autoScoreTopCube = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.TOP_CUBE);
  private final FullScoringCommand m_autoScoreMidCube = new FullScoringCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, m_armPneumaticSubsystem, m_armMotorSubsystem, m_clawPneumaticSubsystem, Constants.MID_CUBE);
  private final AutonomousCommands m_autonomousCommands = new AutonomousCommands();
  private final ReturnToDockCommand m_returnToDockCommand = new ReturnToDockCommand(m_armPneumaticSubsystem, m_armMotorSubsystem);
  private final PrintGetXCommand m_printGetXCommand = new PrintGetXCommand(m_drivetrainSubsystem);
  private final ArmFeedforwardCommand m_armFeedforwardCommand = new ArmFeedforwardCommand(m_armMotorSubsystem);

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
    Trigger printGetX = new JoystickButton(leftJoystick, 10);
    Trigger armFeedforward = new JoystickButton(leftJoystick, 5);

    // Set commmands to button
    recalibrateButton.onTrue(m_resetSwerveGyroCommand);
    balancingButton.onTrue(m_swerveDriveBalanceCommand);
    nonBalancingButton.onTrue(m_swerveDriveCommand);
    stationaryButton.onTrue(m_swerveNoMoveCommand);
    clawPneumaticButton.onTrue(m_clawPneumaticCommand);
    armPneumaticButton.onTrue(m_armPneumaticCommand);
    scoreConeTopLeft.onTrue(m_autoScoreTopLeft);
    scoreConeMidLeft.onTrue(m_autoScoreMidLeft);
    scoreConeTopRight.onTrue(m_autoScoreTopRight);
    scoreConeMidRight.onTrue(m_autoScoreMidRight);
    scoreCubeTop.onTrue(m_autoScoreTopCube);
    scoreCubeMid.onTrue(m_autoScoreMidCube);
    printGetX.onTrue(m_printGetXCommand);
    returnToDock.onTrue(m_returnToDockCommand);
    armFeedforward.onTrue(m_armFeedforwardCommand);
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
