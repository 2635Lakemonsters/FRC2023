// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class GoToAprilTagCommand extends CommandBase {

  private DrivetrainSubsystem m_drivetrainSubsystem;
  private double currentDeg;
  private double errorDeg;

  /** Creates a new GoToAprilTagCommand. */
  public GoToAprilTagCommand(DrivetrainSubsystem drivetrainSubsystem) {
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
    currentDeg = m_drivetrainSubsystem.m_odometry.getPoseMeters().getRotation().getDegrees();
    errorDeg = 180 - currentDeg;
    System.out.println(errorDeg);
    DrivetrainSubsystem.setRotCommanded(RobotContainer.leftJoystick.getX() + errorDeg * 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
