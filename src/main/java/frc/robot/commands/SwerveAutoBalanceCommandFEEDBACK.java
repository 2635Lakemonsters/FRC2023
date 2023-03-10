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
  double ff_gain;
  public double x_feedforward_final; 

  /** Creates a new SwerveDriveBalanceCommand. */
  public SwerveAutoBalanceCommandFEEDBACK(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x_feedforward = m_drivetrainSubsystem.getGyroscope().getRawAccelZ() / Robot.init_gyro_z_accel * ff_gain; 
    double sin = Math.sqrt(1 - x_feedforward * x_feedforward);
    x_feedforward_final = Math.copySign(sin, m_drivetrainSubsystem.getGyroscope().getRoll());
    
    // set the x power commanded
    DrivetrainSubsystem.setXPowerCommanded(RobotContainer.rightJoystick.getY() + x_feedforward);
    DrivetrainSubsystem.setYPowerCommanded(RobotContainer.rightJoystick.getX());
    DrivetrainSubsystem.setRotCommanded(RobotContainer.leftJoystick.getX());
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
