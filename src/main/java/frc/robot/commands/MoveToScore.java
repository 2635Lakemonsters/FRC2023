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
  private double m_length = Constants.LENGTH_OF_BOT;
  private double m_dfo = Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG; //meters
  private double m_targetPoseX;
  private double m_targetPoseY;
  private double m_targetPoseR = 0;
  private int m_scoringPose;
  DrivetrainSubsystem m_driveTrainSubsystem;
  Command m_c = null;

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
    m_allDone = true;
    m_objectTrackerSubsystemChassis.data();
    m_aprilTagData = m_objectTrackerSubsystemChassis.getClosestAprilTag();

    if (m_aprilTagData == null) {
      System.out.println("No AprilTag visible");
      return;
    }

    int tagNumber = m_aprilTagData.getAprilTagID();

    if (util.areWeRed() != util.isRed(tagNumber)) {     // TODO: Coopertition
      System.out.println("Wrong color tag visible");
      return;
    }

    m_allDone = false;
    m_objectTrackerSubsystemChassis.data();
    m_aprilTagData = m_objectTrackerSubsystemChassis.getClosestAprilTag();

    // math to normalize apriltag coordinates to robot-centric coordinates
    double x = m_aprilTagData.x / Constants.INCHES_PER_METER; 
    double y = m_aprilTagData.z / Constants.INCHES_PER_METER;
    
    double thetaOne = Math.atan(x / y);
    double thetaTwo = m_targetPoseR - m_driveTrainSubsystem.m_odometry.getPoseMeters().getRotation().getRadians();
    double thetaThree = (Math.PI / 2)- (thetaOne + thetaTwo);
    double dc = Math.hypot(x, y);
    double lambda = dc * Math.sin(thetaThree);
    double delX = dc * Math.cos(thetaThree);
    double delY = lambda - (m_dfo + (m_length / 2));

    m_targetPoseX = m_driveTrainSubsystem.m_odometry.getPoseMeters().getX() + delX + m_nodeOffset;
    m_targetPoseY = m_driveTrainSubsystem.m_odometry.getPoseMeters().getY() + delY;

    // Simple path without holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
    PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(0.1, 0.1), 
        new PathPoint(mapCoordinates(0.0, 0.0), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
        new PathPoint(mapCoordinates(delX + m_nodeOffset, delY), Rotation2d.fromRadians(thetaTwo) // position, heading(direction of travel)
    ));
    System.out.println("X: " + (delX + m_nodeOffset) + "   delY: " + delY + "   thetaTwo: " + thetaTwo);
    m_c = m_driveTrainSubsystem.followTrajectoryCommand(traj, true);
    m_c.initialize();
  }

  private static Translation2d mapCoordinates(double x, double y) {
    return new Translation2d(-(y / 3), x / 3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_c != null) {
      m_c.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_c != null) {
      m_c.end(interrupted);
    }  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_allDone || m_c == null || m_c.isFinished();
  }
}
