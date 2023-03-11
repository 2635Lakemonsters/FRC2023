// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.drivers.NavX;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SwerveAutoBalanceCommandFEEDBACK extends CommandBase {

  private static DrivetrainSubsystem m_drivetrainSubsystem;
  double ff_gain = 0.0;
  public double x_feedforward_final; 
  double init_roll;

  /** Creates a new SwerveDriveBalanceCommand. */
  public SwerveAutoBalanceCommandFEEDBACK(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x_feedforward = m_drivetrainSubsystem.m_gyro.getRawAccelZ() / Robot.init_gyro_z_accel; 
    double sin = Math.sqrt(1 - x_feedforward * x_feedforward) * ff_gain;
    x_feedforward_final = Math.copySign(sin, m_drivetrainSubsystem.getGyroscope().getRoll());
    
    // set the x power commanded
    double ySpeed = RobotContainer.rightJoystick.getY();
    double xSpeed = RobotContainer.rightJoystick.getX();
    double rotSpeed = RobotContainer.rightJoystick.getTwist();

    ySpeed = Math.copySign(ySpeed * ySpeed * ySpeed, ySpeed);
    xSpeed = Math.copySign(xSpeed * xSpeed * xSpeed, xSpeed);
    rotSpeed = Math.copySign(rotSpeed * rotSpeed * rotSpeed, rotSpeed);

    DrivetrainSubsystem.setXPowerCommanded(-ySpeed + x_feedforward_final);
    DrivetrainSubsystem.setYPowerCommanded(-xSpeed);
    DrivetrainSubsystem.setRotCommanded(-rotSpeed);
    //System.out.println(1.7 * NavX.getXAccelFiltered() - 0.008 * NavX.getRawGyroY());
    System.out.print("Accel Filtered X: "+NavX.getXAccelFiltered());
    System.out.println("\tGyro Y: "+NavX.getRawGyroY());

    /*FF
     * get the z accelerometer component of the gyro and ratio with the force of gravity in robot init -> gives us the cosine of the angle
     * 
     * FB
     * 
     * 54.5 in to get to center
     */
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
