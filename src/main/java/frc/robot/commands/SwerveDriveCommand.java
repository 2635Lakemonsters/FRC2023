// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SwerveDriveCommand extends CommandBase {

  private static DrivetrainSubsystem m_drivetrainSubsystem;
  private static boolean ARE_BALANCING = false;
  private double ff_gain = 0.00;

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     System.out.println("SDC.initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ySpeed = RobotContainer.rightJoystick.getY();
    double xSpeed = RobotContainer.rightJoystick.getX();
    double rotSpeed = RobotContainer.rightJoystick.getTwist();

    // Note that copySign is not needed for cubing.  I've left it in
    // for drivers who prefer squaring.

    ySpeed = Math.copySign(ySpeed * ySpeed * ySpeed, ySpeed);
    xSpeed = Math.copySign(xSpeed * xSpeed * xSpeed, xSpeed);
    rotSpeed = Math.copySign(rotSpeed * rotSpeed * rotSpeed, rotSpeed);

    double x_feedforward_final = 0;
    if (ARE_BALANCING) {
      double x_feedforward = m_drivetrainSubsystem.m_gyro.getRawAccelZ() / Robot.init_gyro_z_accel; 
      double sin = Math.sqrt(1 - x_feedforward * x_feedforward) * ff_gain;
      x_feedforward_final = Math.copySign(sin, m_drivetrainSubsystem.getGyroscope().getRoll());
    }

    // set the x power commanded
    DrivetrainSubsystem.setXPowerCommanded(-ySpeed + x_feedforward_final);
    DrivetrainSubsystem.setYPowerCommanded(-xSpeed);
    DrivetrainSubsystem.setRotCommanded(-rotSpeed);
  }

  public void enableBalance(boolean enable) {
    ARE_BALANCING = enable;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
