// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.models.VisionObject;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class GoToScorePoseCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private ObjectTrackerSubsystem m_objectTrackerSubsystemChassis;
  private VisionObject aprilTagData;

  private double l; // length of robot
  private double dfo = 0.36; //meters
  private double targetPoseX;
  private double targetPoseY;
  private double targetPoseR = 0;
  private int scoringPose;
  
  /** Creates a new GoScoreCommand. */
  public GoToScorePoseCommand(DrivetrainSubsystem drivetrainSubsystem, ObjectTrackerSubsystem objectTrackerSubsystemChassis, int scoringPose) {
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
    
    double thetaOne = Math.atan(x / z);
    double thetaTwo = targetPoseR - m_drivetrainSubsystem.m_odometry.getPoseMeters().getRotation().getRadians();
    double thetaThree = 90 - (thetaOne + thetaTwo);
    double dc = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
    double lambda = dc * Math.sin(thetaThree);

    double delX = dc * Math.cos(thetaThree);
    double delY = lambda - (dfo + (l / 2));

    targetPoseX = m_drivetrainSubsystem.m_odometry.getPoseMeters().getX() + delX;
    targetPoseY = m_drivetrainSubsystem.m_odometry.getPoseMeters().getY() + delY;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPoseX = m_drivetrainSubsystem.m_odometry.getPoseMeters().getX();
    double deltaX = targetPoseX - currentPoseX;

    double currentPoseY = m_drivetrainSubsystem.m_odometry.getPoseMeters().getY();
    double deltaY = targetPoseY - currentPoseY;

    double currentRot = m_drivetrainSubsystem.m_odometry.getPoseMeters().getRotation().getRadians();
    double deltaR = targetPoseR - currentRot;

    // set the max power
    final double gainT = 4.0;
    final double range = 2.0;
    final double gainR = 0.6;

    double lockPoseX = gainT * deltaX;
    lockPoseX = Math.min(Math.max(lockPoseX, -range), range); //clip to range of -2, 2

    double lockPoseY = gainT * deltaY;
    lockPoseY = Math.min(Math.max(lockPoseY, -range), range); //clip to range of -2, 2

    double lockRot = gainR * deltaR;
    lockRot = Math.min(Math.max(lockPoseY, -range), range); //clip to range of -2, 2

    DrivetrainSubsystem.setRotCommanded(RobotContainer.leftJoystick.getX() + lockRot);

    if (scoringPose == 5 || scoringPose == 3) {
      DrivetrainSubsystem.setXPowerCommanded(RobotContainer.rightJoystick.getX() + lockPoseX); //add or subtrack the offset depending on what the robot perceives is adding or subtracting on the feild
      DrivetrainSubsystem.setYPowerCommanded(RobotContainer.rightJoystick.getY() + lockPoseY);
    } else if (scoringPose == 4 || scoringPose == 6) {
      DrivetrainSubsystem.setXPowerCommanded(RobotContainer.rightJoystick.getX() + lockPoseX); //add or subtrack the offset depending on what the robot perceives is adding or subtracting on the feild
      DrivetrainSubsystem.setYPowerCommanded(RobotContainer.rightJoystick.getY() + lockPoseY);
    } else if (scoringPose == 0 || scoringPose == 180) { 
      DrivetrainSubsystem.setXPowerCommanded(RobotContainer.rightJoystick.getX() + lockPoseX);
      DrivetrainSubsystem.setYPowerCommanded(RobotContainer.rightJoystick.getY() + lockPoseY);
    }
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
