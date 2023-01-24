// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.FeedForwardBalancingCommand;
import frc.robot.commands.ResetSwerveGyroCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer extends TimedRobot {

  // Joysticks
  public final Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public final Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  public final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  public final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  public final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Subsystems
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  // Commands
  private final ResetSwerveGyroCommand m_resetSwerveGyroCommand = new ResetSwerveGyroCommand(m_drivetrainSubsystem);
  private final FeedForwardBalancingCommand m_feedForwardBalancingCommand = new FeedForwardBalancingCommand(m_drivetrainSubsystem);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Create button
    Trigger recalibrateButton = new JoystickButton(rightJoystick, Constants.CALIBRATE_BUTTON);
    Trigger balancingButton = new JoystickButton(rightJoystick, Constants.BALANCING_BUTTON);

    // Set commmands to button
    recalibrateButton.onTrue(m_resetSwerveGyroCommand);
    balancingButton.onTrue(m_feedForwardBalancingCommand);
  }

    public Command getAutonomousCommand() {
        return null;
    }
}
