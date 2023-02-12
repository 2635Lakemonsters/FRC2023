// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.models.VisionObject;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class GoScoreCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private ObjectTrackerSubsystem m_objectTrackerSubsystemChassis;
  private VisionObject aprilTagData;

  private double l;
  private double dfo = 0.36; //meters
  private double targetPoseX;
  private double targetPoseY;
  private int scoringPose;
  
  /** Creates a new GoScoreCommand. */
  public GoScoreCommand(DrivetrainSubsystem drivetrainSubsystem, ObjectTrackerSubsystem objectTrackerSubsystemChassis, int scoringPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_objectTrackerSubsystemChassis = objectTrackerSubsystemChassis;

    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_objectTrackerSubsystemChassis.data();
    // Must train drivers / highlight importance to drivers that this will 
    // lock-on to the closest april tag and then it is open loop from here on out.
    //
    // Make sure drivers understand that the assumption is the direct path to the 
    // target is clear of robots / game elements.
    //
    // TODO: Test to see if this is accurate enough to see it once and then 
    //       memorize the target pose and then the robot will go direct-to the target.
    aprilTagData = m_objectTrackerSubsystemChassis.getClosestAprilTag();

    double x = aprilTagData.x;
    double z = aprilTagData.z;
    
    double thetaOne = Math.atan(x / z);
    double thetaTwo = 0;
    double thetaThree = 90 - (thetaOne + thetaTwo);
    double dc = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
    double lambda = dc * Math.sin(thetaThree);

    // Not sure if this logic for calculating these delX delY belongs here in the init, or in 
    // a MissionObject or VisionObject.
    double delX = dc * Math.cos(thetaThree);
    double delY = lambda - (dfo + (l / 2));

    // this sets the target pose of where we want to go, but does not apply any offsets for any 
    // of the reletive positions where we actually want to go with respect to the april tags.
    targetPoseX = m_drivetrainSubsystem.m_odometry.getPoseMeters().getX() + delX;
    targetPoseY = m_drivetrainSubsystem.m_odometry.getPoseMeters().getY() + delY;

    // ...and assumption is that the target heading will be hard-coded per target location.
    // notably all of them are facing one end of the field, but there is one target pick-up location
    // which is not facing the same as all of the other target locations.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPoseX = m_drivetrainSubsystem.m_odometry.getPoseMeters().getX();
    double deltaX = targetPoseX - currentPoseX;

    double currentPoseY = m_drivetrainSubsystem.m_odometry.getPoseMeters().getX();
    double deltaY = targetPoseY - currentPoseY;

    // // what about yaw / heading targets?
    // double currentPoseR = something here to get yaw... yaw is kind of complicated.
    // double deltaR = targetPoseR - currentPoseR;

    // set the max power
    final double gain = 4.0;
    final double range = 2.0;

    double lockPoseX = gain * deltaX;
    lockPoseX = Math.min(Math.max(lockPoseX, -range), range); //clip to range of -2, 2

    double lockPoseY = gain * deltaY;
    lockPoseY = Math.min(Math.max(lockPoseY, -range), range); //clip to range of -2, 2

    // OK, I see what you are doing here... modeling after the SwerveNoMoveCommand
    // This is fine... note however, the SwerveNoMoveCommand was based on the fact that your 
    // initial pose/location was pretty close to the target.  As such, you would not get a large command.
    // Here, since we are telling the robot to go to some other location on the field, the initial command 
    // will be large and will drive it to 100% speed... really need that trapezoidal movement.
    if (scoringPose == 5 || scoringPose == 3) {
      DrivetrainSubsystem.setXPowerCommanded(RobotContainer.rightJoystick.getX() + lockPoseX); //add or subtrack the offset depending on what the robot perceives is adding or subtracting on the feild
      DrivetrainSubsystem.setYPowerCommanded(RobotContainer.rightJoystick.getY() + lockPoseY);
    } else if (scoringPose == 4 || scoringPose == 6) {
      DrivetrainSubsystem.setXPowerCommanded(RobotContainer.rightJoystick.getX() + lockPoseX); //add or subtrack the offset depending on what the robot perceives is adding or subtracting on the feild
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
