/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.models.VisionObject;
import frc.robot.subsystems.DrivetrainSubsystem;

// import com.kauailabs.navx.frc.AHRS;  
// import edu.wpi.first.wpilibj.SPI;

/* 
  Copy of FetchPowerCellCommand with modified contructor to take cargo color
*/

public class VisionDriveClosedLoopCommand extends CommandBase {
  DrivetrainSubsystem m_driveTrainSubsystem;

  PIDController angleController;
  PIDController strafeController;
  PIDController forwardController; 
  double gyroAngle;
  double angle; 
  double desiredAngle;
  double setPointAngle = 6;
  boolean isClose;

  String targetObjectLabel; // cone, cube, or AprilTag
  int aprilTagID;

  public double totalRotation = 0;

  private boolean inTeleop = false; 

  public VisionDriveClosedLoopCommand(String targetObjectLabel, DrivetrainSubsystem ds) {
    //initPID();
    checkUpdateObjectLabel(targetObjectLabel);     
    m_driveTrainSubsystem = ds;
    addRequirements(m_driveTrainSubsystem);
  }

  public VisionDriveClosedLoopCommand(String targetObjectLabel, boolean inTeleop, DrivetrainSubsystem ds) {
    //initPID();
    checkUpdateObjectLabel(targetObjectLabel); 
    this.inTeleop = inTeleop;    
    m_driveTrainSubsystem = ds;
    addRequirements(m_driveTrainSubsystem);
  }

  private void checkUpdateObjectLabel(String label) {
    // checks targetObjectLabel given to constructor, normalizes case
    if (label.equalsIgnoreCase("cone")) {
      this.targetObjectLabel = "cone";

    } else if (label.equalsIgnoreCase("cube")) {
      this.targetObjectLabel = "cube"; 

    } else if (label.contains("tag16h5")) {
      this.targetObjectLabel = "aprilTag";
      this.aprilTagID = Integer.valueOf(label.substring(label.indexOf(" "),label.length()-1)); 
    }
  }

  protected void initPID(){
    angleController = new PIDController(0.25, 0.0, 0.0);
    strafeController = new PIDController(0.011, 0.0, 0.0); // TODO update constants
    forwardController = new PIDController(0.05, 0.01, 0.0); // TODO update constants   
  }

  @Override
  public void initialize() {
    initPID();
    System.out.println("FCC start");
    
    // SmartDashboard.putNumber("Vision angle", angle);
    // SmartDashboard.putNumber("Desired angle", desiredAngle);
    // SmartDashboard.putNumber("initial angle", gyroAngle);
    // SmartDashboard.putNumber("SetPoint angle", setPointAngle);
    
    Vector2 position = new Vector2(0, 0);
    Robot.drivetrainSubsystem.resetKinematics(position, 0);
    System.out.println("Initialized FCC");

    isClose = false;
    SmartDashboard.putString("FCC Status", "FCC Init");
  }

  @Override
  public void execute() {
    
    Robot.objectTrackerSubsystem.data();
    double forward = 0;
    double strafe = 0;
    double rotation = 0;

    VisionObject closestObject = Robot.objectTrackerSubsystem.getClosestObject(targetObjectLabel);
     
    if (closestObject == null || closestObject.z > 150) {
      SmartDashboard.putNumber("driveRotation", 99);
      Robot.drivetrainSubsystem.holonomicDrive(new Vector2(0,0), 0, false);
      SmartDashboard.putString("FCC Status", "No cargo in frame OR cargo out of range");
      return; // no object found
    }
    
    double v; // velocity? 3/14
    // System.out.println("Closest z: " + closestObject.z);
    closestObject.motionCompensate(Robot.drivetrainSubsystem, true);
  
    double angle =  Math.atan2(closestObject.x, closestObject.z);
    
    angleController.setSetpoint(angle);
    rotation = angleController.calculate(0) * 0;
    
    if (rotation > 1){
      rotation = 1;
    } else if (rotation < -1){
      rotation = -1;
    }

    totalRotation += rotation;
    SmartDashboard.putNumber("driveRotation", rotation);
    
    // strafe
    strafeController.setSetpoint(closestObject.x);
    strafe = strafeController.calculate(0);

    if(strafe > 1){
      strafe = 1;
    }else if (strafe < -1){
      strafe = -1;
    }

    SmartDashboard.putNumber("driveStrafe", strafe);

    // forward
    //forwardController.setSetpoint(closestObject.z-RobotMap.TARGET_TRIGGER_DISTANCE); // TODO figure out how to implement code that begins intake process 
    forward = forwardController.calculate(0);

    if(forward > 1){
      forward = 1;
    }else if (forward < -1){
      forward = -1;
    }

    SmartDashboard.putNumber("driveForward", forward);
    
    final boolean robotOriented = false;

    //final Vector2 translation = new Vector2(-forward, -strafe*0);
  
    v = -0.4;  

    if (inTeleop) {
      v = -0.7;
    }
    
    //  if (closestObject.z < 60) {
    //    isClose = true;
    // //   v = -0.05;
     
    //  }
    final Vector2 translation = new Vector2(v, strafe);
    // System.out.println("translation: " + translation);
    Robot.drivetrainSubsystem.holonomicDrive(translation, rotation, robotOriented);
  }

@Override
public boolean isFinished() {
  double tolerance = 4; // TODO units...? i think it's inches
  VisionObject closestObject = Robot.objectTrackerSubsystem.getClosestObject(targetObjectLabel);
  if(closestObject == null) {
    return false;
  }//TODO could lose sight for small amount of time causing command to finish early
  
  //boolean done = Math.abs(closestObject.z-RobotMap.TARGET_TRIGGER_DISTANCE) <= tolerance;
  isClose = Math.abs(closestObject.z - RobotMap.TARGET_TRIGGER_DISTANCE) <= tolerance;
  // if (done) {
  //   System.out.println("done FCC");
  // }
  if (isClose) {
    System.out.println("FCC done");
  }
  return isClose;
  // return false;
  // boolean isFinished = super.isTimedOut();
  // if (isFinished) {
  //   SmartDashboard.putNumber("totalRotation", totalRotation);
  // }
  //  return isFinished;
     // TODO: add the actual completion test code
}

  @Override
  public void end(boolean interrupted) {
    Robot.vision.ledOff();
    // Robot.drivetrainSubsystem.holonomicDrive(new Vector2(-100.0, 0.0), 0, true);
    // System.out.println("FCC end() drive forward extra 5 in");

    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0, true);
    System.out.println("FCC end()");
  }

}