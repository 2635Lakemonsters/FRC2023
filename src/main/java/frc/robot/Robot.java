// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Calendar;

import javax.print.attribute.standard.MediaSize.NA;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.drivers.NavX;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SwerveDriveCommand(RobotContainer.m_drivetrainSubsystem,2);
    m_robotContainer.m_drivetrainSubsystem.updateOdometry();
    NavX.updateXAccelFiltered();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  /** This function is called periodically during operator control. */

  private long last_time;
  @Override
  public void teleopPeriodic() {

    Calendar calendar = Calendar.getInstance();
    //Returns current time in millis
      
    m_robotContainer.m_drivetrainSubsystem.m_frontLeft.updateSwerveTable(); // 0 analog ID 
    m_robotContainer.m_drivetrainSubsystem.m_frontRight.updateSwerveTable(); // 3 analog ID
    m_robotContainer.m_drivetrainSubsystem.m_backLeft.updateSwerveTable(); // 1 analog ID
    m_robotContainer.m_drivetrainSubsystem.m_backRight.updateSwerveTable(); //2 analog ID

    SmartDashboard.putNumber("x accel", NavX.getRawAccelX());
    SmartDashboard.putNumber("y accel", NavX.getRawAccelY());
    SmartDashboard.putNumber("z accel", NavX.getRawAccelZ());
    SmartDashboard.putNumber("x gyro", NavX.getRawGyroX());
    SmartDashboard.putNumber("y gyro", NavX.getRawGyroY());
    SmartDashboard.putNumber("z gyro", NavX.getRawGyroZ());
    SmartDashboard.putNumber("kXAccel", m_robotContainer.rightJoystick.getY() / NavX.getRawAccelX());
    SmartDashboard.putNumber("kRoll", m_robotContainer.rightJoystick.getY() / NavX.getRoll());
    SmartDashboard.putNumber("kGyro Y", m_robotContainer.rightJoystick.getY() / NavX.getRawGyroY());
    SmartDashboard.putNumber("navX pitch", NavX.getPitch());
    SmartDashboard.putNumber("navX roll", NavX.getRoll());
    SmartDashboard.putNumber("navX yaw", NavX.getYaw());
    SmartDashboard.putNumber("k * raw gyro y", 0.5 * NavX.getRawGyroY());
    SmartDashboard.putNumber("kPitch", m_robotContainer.rightJoystick.getY() / NavX.getPitch());

    BuiltInAccelerometer rioAccel = new BuiltInAccelerometer();
    SmartDashboard.putNumber("RoboRio x accel", rioAccel.getX());
    SmartDashboard.putNumber("RoboRio y accel", rioAccel.getY());
    SmartDashboard.putNumber("RoboRio z accel", rioAccel.getZ());

    // SmartDashboard.putNumber("clock cycle", calendar.getTimeInMillis() - last_time);
    // kXAccel pose to hold pose is 0.48
    // 1.0 will move it back at a mid rate

    NavX.updateXAccelFiltered();
    last_time = calendar.getTimeInMillis();
  }

  private void SwerveDriveCommand(DrivetrainSubsystem mDrivetrainsubsystem, int i) {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_robotContainer.m_drivetrainSubsystem.m_frontLeft.updateSwerveTable(); // 0 analog ID 
    m_robotContainer.m_drivetrainSubsystem.m_frontRight.updateSwerveTable(); // 3 analog ID
    m_robotContainer.m_drivetrainSubsystem.m_backLeft.updateSwerveTable(); // 1 analog ID
    m_robotContainer.m_drivetrainSubsystem.m_backRight.updateSwerveTable(); //2 analog ID
    System.out.println("front left table ang: " + m_robotContainer.m_drivetrainSubsystem.m_frontLeft.t_turningEncoder.getDouble(-1));
    System.out.println("front right table ang: " + m_robotContainer.m_drivetrainSubsystem.m_frontRight.t_turningEncoder.getDouble(-1));
    System.out.println("back left table ang: " + m_robotContainer.m_drivetrainSubsystem.m_backLeft.t_turningEncoder.getDouble(-1));
    System.out.println("back right table ang: " + m_robotContainer.m_drivetrainSubsystem.m_backRight.t_turningEncoder.getDouble(-1));
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
