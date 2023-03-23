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

/** Uses AprilTags to generate trajectory. Open-loop / dead-reckoning */
public class MoveToScore extends CommandBase {

  private final double moveCloserDistance = 60;

  private double m_nodeOffset;
  private ObjectTrackerSubsystem m_objectTrackerSubsystemChassis;
  private VisionObject m_aprilTagData;
  private boolean m_allDone = false;
  // private double m_length = Constants.LENGTH_OF_BOT;
  private double m_dfo; //meters
  // private double m_targetPoseR = (Math.PI / 2);
  DrivetrainSubsystem m_driveTrainSubsystem;
  Command m_c = null;
  boolean m_moveCloser;

  /** Creates a new MoveToScore. */
  public MoveToScore(DrivetrainSubsystem driveTrainSubsystem, ObjectTrackerSubsystem objectTrackerSubsystemChassis, double nodeOffset, double fieldOffset, boolean moveCloser) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dfo = fieldOffset;
    m_nodeOffset = nodeOffset;
    m_objectTrackerSubsystemChassis = objectTrackerSubsystemChassis;
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_moveCloser = moveCloser;
  }

  public MoveToScore(DrivetrainSubsystem driveTrainSubsystem, ObjectTrackerSubsystem objectTrackerSubsystemChassis, double nodeOffset, double fieldOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dfo = fieldOffset;
    m_nodeOffset = nodeOffset;
    m_objectTrackerSubsystemChassis = objectTrackerSubsystemChassis;
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_moveCloser = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_c = null;
    m_allDone = true;
    m_aprilTagData = null;
    System.out.println("about to call ots.data");
    m_objectTrackerSubsystemChassis.data();
    System.out.println("ots.data complete");
    m_aprilTagData = m_objectTrackerSubsystemChassis.getClosestAprilTag();

    if (m_aprilTagData == null) {
      System.out.println("No AprilTag visible");
      return;
    }

    int tagNumber = m_aprilTagData.getAprilTagID();

    if (util.areWeRed() != util.isRed(tagNumber)) {
      System.out.println("Wrong color tag visible");
      return;
    }

    System.out.println("Found a tag");

    m_allDone = false;
    double x, y;

    try {
      // math to normalize apriltag coordinates to robot-centric coordinates
      x = m_aprilTagData.x / Constants.INCHES_PER_METER; 
      y = m_aprilTagData.z / Constants.INCHES_PER_METER;
      // double ya = Math.toRadians(m_aprilTagData.ya);
    } catch (Exception e) {
      System.out.println("APRILTAG X OR Y IS NULL");
      return;
    }

    // if ((y * Constants.INCHES_PER_METER) > 150) { // to make sure it doesn't move too early
    //   System.out.println("Too far away");
    //   return;
    // }

    if (m_moveCloser && (y * Constants.INCHES_PER_METER) < moveCloserDistance)
    {
      return;             // Already < moveCloserDistance inches away.  No need to move closer
    }
    
    double delY;

    if (m_moveCloser)
    {
      delY = y - moveCloserDistance/Constants.INCHES_PER_METER;     // If further than 50 inches, move to 50 inches
    }
    else{
      delY = y - m_dfo - Constants.BUMPER_THICKNESS;
    }
    double delX = m_nodeOffset + x;

    PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(2, 0.5), 
        new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
        new PathPoint(new Translation2d(-delY, delX), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0) // position, heading(direction of travel)
    ));
    System.out.println("X: " + x + "   Y: " + y);
    System.out.println("delX: " + delX + "   delY: " + delY);
    m_c = m_driveTrainSubsystem.followTrajectoryCommand(traj, true);
    m_c.initialize();
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
