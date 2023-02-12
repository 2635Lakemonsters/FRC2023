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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ArmPneumaticCommand;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.ClawPneumaticCommand;
import frc.robot.commands.GoScoreCommand;
import frc.robot.commands.ResetSwerveGyroCommand;
import frc.robot.commands.SwerveAutoBalanceCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveNoMoveCommand;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import frc.robot.subsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class RobotContainer extends TimedRobot {
  // Joysticks
  public final static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public final static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  public static final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  public static final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  public static final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Subsystems
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static final ClawPneumaticSubsystem m_clawPneumaticSubsystem = new ClawPneumaticSubsystem();
  public static final ArmPneumaticSubsystem m_armPneumaticSubsystem = new ArmPneumaticSubsystem();
  public static final ObjectTrackerSubsystem m_objectTrackerSubsystemGripper = new ObjectTrackerSubsystem("Gripper");
  public static final ObjectTrackerSubsystem m_objectTrackerSubsystemChassis = new ObjectTrackerSubsystem("Chassis");

  // Commands
  private final ResetSwerveGyroCommand m_resetSwerveGyroCommand = new ResetSwerveGyroCommand(m_drivetrainSubsystem);
  private final SwerveDriveCommand m_swerveDriveCommand = new SwerveDriveCommand(m_drivetrainSubsystem);
  private final SwerveAutoBalanceCommand m_swerveDriveBalanceCommand = new SwerveAutoBalanceCommand(m_drivetrainSubsystem);
  private final SwerveNoMoveCommand m_swerveNoMoveCommand = new SwerveNoMoveCommand(m_drivetrainSubsystem);
  private final ClawPneumaticCommand m_clawPneumaticCommand = new ClawPneumaticCommand(m_clawPneumaticSubsystem);
  private final ArmPneumaticCommand m_armPneumaticCommand = new ArmPneumaticCommand(m_armPneumaticSubsystem);
  private final GoScoreCommand m_scoreConeTopLeft = new GoScoreCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.TOP_LEFT_CONE);
  private final GoScoreCommand m_scoreConeMidLeft = new GoScoreCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.MID_LEFT_CONE);
  private final GoScoreCommand m_scoreConeTopRight = new GoScoreCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.TOP_RIGHT_CONE);
  private final GoScoreCommand m_scoreConeMidRight = new GoScoreCommand(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.MID_RIGHT_CONE);
  private final AutonomousCommands m_autonomousCommands = new AutonomousCommands();

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
    Trigger scoreConeTopLeft = new JoystickButton(leftJoystick, Constants.TOP_LEFT_CONE);
    Trigger scoreConeMidLeft = new JoystickButton(leftJoystick, Constants.MID_LEFT_CONE);
    Trigger scoreConeTopRight = new JoystickButton(leftJoystick, Constants.TOP_RIGHT_CONE);
    Trigger scoreConeMidRight = new JoystickButton(leftJoystick, Constants.MID_RIGHT_CONE);

    // Set commmands to button
    recalibrateButton.onTrue(m_resetSwerveGyroCommand);
    balancingButton.onTrue(m_swerveDriveBalanceCommand);
    nonBalancingButton.onTrue(m_swerveDriveCommand);
    stationaryButton.onTrue(m_swerveNoMoveCommand);
    clawPneumaticButton.onTrue(m_clawPneumaticCommand);
    armPneumaticButton.onTrue(m_armPneumaticCommand);
    scoreConeTopLeft.onTrue(m_scoreConeTopLeft);
    scoreConeMidLeft.onTrue(m_scoreConeMidLeft);
    scoreConeTopRight.onTrue(m_scoreConeTopRight);
    scoreConeMidRight.onTrue(m_scoreConeMidRight);
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
