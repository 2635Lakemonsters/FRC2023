// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.util;
import frc.robot.models.VisionObject;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class MoveToScore extends CommandBase {

  private double m_nodeOffset;
  private ObjectTrackerSubsystem m_objectTrackerSubsystemChassis;
  private VisionObject m_aprilTagData;
  private boolean m_allDone = false;
  private double m_length; // TODO: length of robot
  private double m_dfo = 0.36; //meters
  private double m_targetPoseX;
  private double m_targetPoseY;
  private double m_targetPoseR = 0;
  private int m_scoringPose;
  DrivetrainSubsystem m_driveTrainSubsystem;
  Command m_c;

  /** Creates a new MoveToScore. */
  public MoveToScore(DrivetrainSubsystem driveTrainSubsystem, ObjectTrackerSubsystem objectTrackerSubsystemChassis, double nodeOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_nodeOffset = nodeOffset;
    m_objectTrackerSubsystemChassis = objectTrackerSubsystemChassis;
    m_driveTrainSubsystem = driveTrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_objectTrackerSubsystemChassis.data();
    m_aprilTagData = m_objectTrackerSubsystemChassis.getClosestAprilTag();
    int tagNumber = m_aprilTagData.getAprilTagID();
    m_allDone = false;

    if (util.areWeRed() != util.isRed(tagNumber)) {     // TODO: Coopertition
      m_allDone = true;
      return;
    }

    m_objectTrackerSubsystemChassis.data();
    m_aprilTagData = m_objectTrackerSubsystemChassis.getClosestAprilTag();

    double x = m_aprilTagData.x;
    double z = m_aprilTagData.z;
    
    double thetaOne = Math.atan(x / z);
    double thetaTwo = m_targetPoseR - m_driveTrainSubsystem.m_odometry.getPoseMeters().getRotation().getRadians();
    double thetaThree = 90 - (thetaOne + thetaTwo);
    double dc = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
    double lambda = dc * Math.sin(thetaThree);

    double delX = dc * Math.cos(thetaThree);
    double delY = lambda - (m_dfo + (m_length / 2));

    m_targetPoseX = m_driveTrainSubsystem.m_odometry.getPoseMeters().getX() + delX + m_nodeOffset;
    m_targetPoseY = m_driveTrainSubsystem.m_odometry.getPoseMeters().getY() + delY;

// Simple path without holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
    PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(4, 3), 
        new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
        new PathPoint(new Translation2d(delX + m_nodeOffset, delY), Rotation2d.fromRadians(thetaTwo) // position, heading(direction of travel)
    ));
    m_c = m_driveTrainSubsystem.followTrajectoryCommand(traj, true);
    m_c.initialize();
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_c.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_c.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_allDone || m_c.isFinished();
  }
}
