package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.drivers.NavX;


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

  public static double init_gyro_z_accel; 
  public static double init_roll;

  boolean m_didViolate = false; 

  public static double initialGravityZAccel;
  public static double initGyroRoll;

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
    RobotContainer.m_drivetrainSubsystem.zeroOdometry();
    m_autoChooser = m_robotContainer.getAutonomousCommand();
    RobotContainer.m_drivetrainSubsystem.recalibrateGyro();

    this.init_gyro_z_accel = RobotContainer.m_drivetrainSubsystem.getGyroscope().getRawAccelZ();
    init_roll = RobotContainer.m_drivetrainSubsystem.getGyroscope().getRoll();


    System.out.println("Initial Rotation: " + RobotContainer.m_drivetrainSubsystem.m_odometry.getPoseMeters().getRotation().getDegrees());

    PathPlannerServer.startServer(5811);

    initialGravityZAccel = RobotContainer.m_drivetrainSubsystem.getGyroscope().getRawAccelZ();
    initGyroRoll = RobotContainer.m_drivetrainSubsystem.getGyroscope().getRoll();
    
    // close claw so that you know for sure that the state of the pneumatic controller reflects the state of the pneumatic
    RobotContainer.m_clawPneumaticSubsystem.grabberClose();
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
    m_robotContainer.m_resetSwerveGyroCommand.execute();

    m_autonomousCommand = m_autoChooser.getSelected();
    
    // print camera values to log
    RobotContainer.m_objectTrackerSubsystemChassis.data();
    RobotContainer.m_objectTrackerSubsystemGripper.data();

    System.out.println("AUTO INIT");
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // RobotContainer.m_drivetrainSubsystem.updateOdometry();
    NavX.updateXAccelFiltered();
  }

  @Override
  public void teleopInit() {
    RobotContainer.m_drivetrainSubsystem.zeroOdometry();
    // m_robotContainer.m_resetSwerveGyroCommand.execute();
    RobotContainer.m_drivetrainSubsystem.followJoystick();
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

    // boolean isViolating = false;
    // boolean isExtended = RobotContainer.m_armPneumaticSubsystem.getIsExtended();
    // double theta = RobotContainer.m_armMotorSubsystem.getTheta();

    NavX.updateXAccelFiltered();
  }

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
