// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.models.VisionObject;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class GoToAprilTagCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private ObjectTrackerSubsystem m_objectTrackerSubsystemChassis;
  private VisionObject aprilTagData;
  private double thetaOne;
  private double thetaTwo;
  private double thetaThree;
  private double dc;
  private double delX;
  private double delY;
  private double lambda;
  private double l;
  private double dfo;
  
  /** Creates a new GoToAprilTagCommand. */
  public GoToAprilTagCommand(DrivetrainSubsystem drivetrainSubsystem, ObjectTrackerSubsystem objectTrackerSubsystemChassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_objectTrackerSubsystemChassis = objectTrackerSubsystemChassis;

    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_objectTrackerSubsystemChassis.data();
    aprilTagData = m_objectTrackerSubsystemChassis.getClosestAprilTag();

    double x = aprilTagData.x;
    double z = aprilTagData.z;
    
    thetaOne = Math.atan(x / z);
    thetaThree = 90 - (thetaOne + thetaTwo);
    dc = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
    delX = dc * Math.cos(thetaThree);
    lambda = dc * Math.sin(thetaThree);
    delY = lambda - (dfo + (l / 2));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DrivetrainSubsystem.setXPowerCommanded(RobotContainer.rightJoystick.getY());
    DrivetrainSubsystem.setYPowerCommanded(RobotContainer.rightJoystick.getX());
    DrivetrainSubsystem.setRotCommanded(RobotContainer.leftJoystick.getX());
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
