// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivers.NavX;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SwerveNoMoveCommand extends CommandBase {

  private static DrivetrainSubsystem m_drivetrainSubsystem;
  private double targetPoseX;


  /** Creates a new SwerveNoMoveCommand. */
  public SwerveNoMoveCommand(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPoseX = m_drivetrainSubsystem.m_odometry.getPoseMeters().getX();
    System.out.println("targetPoseX: " + targetPoseX);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPoseX = m_drivetrainSubsystem.m_odometry.getPoseMeters().getX();
    double delta = targetPoseX - currentPoseX;
    System.out.println("delta: " + delta);
    System.out.println("currentPoseX: " + currentPoseX);

    // set the x power commanded
    final double gain = 4.0;
    final double range = 2.0;
    double lockPose = gain * delta;
    // System.out.println(lockPose);
    lockPose = Math.min(Math.max(lockPose, -range), range); //clip to range of -4, 4
    // System.out.println(lockPose);
    DrivetrainSubsystem.setXPowerCommanded(lockPose + 0.6 * NavX.getXAccelFiltered() - 0.005 * NavX.getRawGyroY());
    // DrivetrainSubsystem.setYPowerCommanded(RobotContainer.rightJoystick.getX());
    // DrivetrainSubsystem.setRotCommanded(RobotContainer.leftJoystick.getX());
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
