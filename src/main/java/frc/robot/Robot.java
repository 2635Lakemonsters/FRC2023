// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.drivers.NavX;
import frc.robot.subsystems.DrivetrainSubsystem;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private SendableChooser<Command> m_autoChooser; 
  // private BuiltInAccelerometer rioAccel = new BuiltInAccelerometer();

  // for motion compensate (vision)
  public static int circularBufferSize = 50;
  public static int bufferSlotNumber = 0;
  public static double[] time;
  public static double[] angle;
  boolean autoHappened; 

  boolean m_didViolate = false; 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // motion compensate (vision)
    time = new double[circularBufferSize]; 
    angle =  new double[circularBufferSize];
    autoHappened = false;

    m_robotContainer = new RobotContainer();
    m_robotContainer.m_resetSwerveGyroCommand.execute();

    System.out.println("Initial Rotation: " + RobotContainer.m_drivetrainSubsystem.m_odometry.getPoseMeters().getRotation().getDegrees());

    PathPlannerServer.startServer(5811);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.m_drivetrainSubsystem.zeroOdometry();
    m_autoChooser = m_robotContainer.getAutonomousCommand();

    m_autonomousCommand = m_autoChooser.getSelected();
    
    // print camera values to log
    RobotContainer.m_objectTrackerSubsystemChassis.data();
    RobotContainer.m_objectTrackerSubsystemGripper.data();
    // chassis_z and grip_r not working 2/13
    // double chassis_z = RobotContainer.m_objectTrackerSubsystemChassis.getClosestObject("cube").z;
    // double grip_r = RobotContainer.m_objectTrackerSubsystemGripper.getClosestObject("cone").r;
    // double april_x = RobotContainer.m_objectTrackerSubsystemChassis.getClosestAprilTag().x;
    // int april_id = RobotContainer.m_objectTrackerSubsystemChassis.getClosestAprilTag().getAprilTagID();
    
    // System.out.println(chassis_z);
    // System.out.println(grip_r);
    // System.out.println(RobotContainer.m_objectTrackerSubsystemChassis.foundObjects[0].toString());
    // System.out.println(RobotContainer.m_objectTrackerSubsystemGripper.foundObjects);

    // System.out.println(april_x);
    // System.out.println(april_id);

    System.out.println("AUTO INIT");

    // m_autonomousCommand = new AutonomousTrajectoryCommand(RobotContainer.m_drivetrainSubsystem).runAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SwerveDriveCommand(RobotContainer.m_drivetrainSubsystem,2);
    RobotContainer.m_drivetrainSubsystem.updateOdometry();
    NavX.updateXAccelFiltered();
  }

  @Override
  public void teleopInit() {
    RobotContainer.m_drivetrainSubsystem.zeroOdometry();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // TODO add something to put arm in default state - may not need this in autoinit because arm starts in known pos before auto
    
  }

  /** This function is called periodically during operator control. */

  @Override
  public void teleopPeriodic() {
    //Returns current time in millis

    // SmartDashboard.putNumber("x accel", NavX.getRawAccelX());
    // SmartDashboard.putNumber("y accel", NavX.getRawAccelY());
    // SmartDashboard.putNumber("z accel", NavX.getRawAccelZ());
    // SmartDashboard.putNumber("x gyro", NavX.getRawGyroX());
    // SmartDashboard.putNumber("y gyro", NavX.getRawGyroY());
    // SmartDashboard.putNumber("z gyro", NavX.getRawGyroZ());
    // SmartDashboard.putNumber("kXAccel", RobotContainer.rightJoystick.getY() / NavX.getRawAccelX());
    // SmartDashboard.putNumber("kRoll", RobotContainer.rightJoystick.getY() / NavX.getRoll());
    // SmartDashboard.putNumber("kGyro Y", RobotContainer.rightJoystick.getY() / NavX.getRawGyroY());
    // SmartDashboard.putNumber("navX pitch", NavX.getPitch());
    // SmartDashboard.putNumber("navX roll", NavX.getRoll());
    // SmartDashboard.putNumber("navX yaw", NavX.getYaw());
    // SmartDashboard.putNumber("k * raw gyro y", 0.5 * NavX.getRawGyroY());
    // SmartDashboard.putNumber("kPitch", RobotContainer.rightJoystick.getY() / NavX.getPitch());
    // SmartDashboard.putNumber("RoboRio x accel", rioAccel.getX());
    // SmartDashboard.putNumber("RoboRio y accel", rioAccel.getY());
    // SmartDashboard.putNumber("RoboRio z accel", rioAccel.getZ());

    // SmartDashboard.putNumber("clock cycle", calendar.getTimeInMillis() - last_time);
    // kXAccel pose to hold pose is 0.48
    // 1.0 will move it back at a mid rate

    boolean isViolating = false;
    boolean isExtended = RobotContainer.m_armPneumaticSubsystem.getIsExtended();
    double theta = RobotContainer.m_armMotorSubsystem.getTheta();

    if (!isExtended && (theta > 172 && theta < 208)) { // vertical
      isViolating = true;
      SmartDashboard.putNumber("violated vertical", isViolating ? 1 : 0);
      m_didViolate = true;

    } else if (isExtended && theta < 270 && theta > 218) { // horizontal
      isViolating = true;
      SmartDashboard.putNumber("violated horizontal", isViolating ? 1 : 0);
      m_didViolate = true;

    } else {
      SmartDashboard.putNumber("violated vertical", isViolating ? 1 : 0);
      SmartDashboard.putNumber("violated horizontal", isViolating ? 1 : 0);
      SmartDashboard.putNumber("DIDNT VIOLATE WHOO", isViolating ? 1 : 0);
    }
    
    SmartDashboard.putNumber("isViolating", (isViolating ? 1 : 0));
    SmartDashboard.putBoolean("didViolate at some point", m_didViolate);

    NavX.updateXAccelFiltered();
  }

  private void SwerveDriveCommand(DrivetrainSubsystem mDrivetrainsubsystem, int i) {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
