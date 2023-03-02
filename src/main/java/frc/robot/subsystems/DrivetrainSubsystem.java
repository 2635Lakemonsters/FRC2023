// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PPLogging;
import frc.robot.RobotContainer;
import frc.robot.drivers.NavX;

public class DrivetrainSubsystem extends SubsystemBase {

    public static final double kMaxSpeed = 3.63; // 3.63 meters per second
    // public final double kMaxSpeed = 0.5;
    public final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  
    public final double m_drivetrainWheelbaseWidth = 18.5 / Constants.INCHES_PER_METER;
    public final double m_drivetrainWheelbaseLength = 28.5 / Constants.INCHES_PER_METER;

    public final Translation2d m_frontLeftLocation = 
            new Translation2d(-m_drivetrainWheelbaseWidth/2, -m_drivetrainWheelbaseLength/2);
    public final Translation2d m_frontRightLocation = 
            new Translation2d(-m_drivetrainWheelbaseWidth/2, m_drivetrainWheelbaseLength/2);
    public final Translation2d m_backLeftLocation = 
            new Translation2d(m_drivetrainWheelbaseWidth/2, -m_drivetrainWheelbaseLength/2);
    public final Translation2d m_backRightLocation = 
            new Translation2d(m_drivetrainWheelbaseWidth/2, m_drivetrainWheelbaseLength/2);

    public final SwerveModule m_frontLeft = new SwerveModule(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER, 
                                                              Constants.FRONT_LEFT_ANGLE_OFFSET_COMPETITION);
    public final SwerveModule m_frontRight = new SwerveModule(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER, 
                                                              Constants.FRONT_RIGHT_ANGLE_OFFSET_COMPETITION);
    public final SwerveModule m_backLeft = new SwerveModule(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER, 
                                                              Constants.BACK_LEFT_ANGLE_OFFSET_COMPETITION);
    public final SwerveModule m_backRight = new SwerveModule(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER, 
                                                              Constants.BACK_RIGHT_ANGLE_OFFSET_COMPETITION);
  
    // private final AnalogGyro m_gyro = new AnalogGyro(0);
  
    private final NavX m_gyro = new NavX(SPI.Port.kMXP);
  
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation,
      m_frontRightLocation, 
      m_backLeftLocation, 
      m_backRightLocation);
  
    public final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
            });

    // pid constants from 2022 FOLLOWER_TRANSLATION_CONSTANTS and FOLLOWER_ROTATION_CONSTANTS
    private static final double TRANSLATION_P = 0.018; //0.05
    private static final double TRANSLATION_I = 0.0; //0.01
    private static final double TRANSLATION_D = 0.0;
    private static final double ROTATION_P = 0.0; //0.3
    private static final double ROTATION_I = 0.0; //0.01
    private static final double ROTATION_D = 0.0;
    

  /** Creates a new DrivetrianSubsystem. */
  public DrivetrainSubsystem() {
    m_gyro.calibrate();
    getPose();
  }

  private static double xPowerCommanded = 0;
  private static double yPowerCommanded = 0;
  private static double rotCommanded = 0;

  // three setters here and then call the setteres from the sd execute
  public static void setXPowerCommanded(double xPower) {
    xPowerCommanded = xPower;
  }

  public static void setYPowerCommanded(double yPower) {
    yPowerCommanded = yPower;
  }

  public static void setRotCommanded(double rot) {
    rotCommanded = rot;
  }

  @Override
  public void periodic() {
    putDTSToSmartDashboard();

    // Get the x speed
    final var xPower =
      RobotContainer.m_xspeedLimiter.calculate(MathUtil.applyDeadband(xPowerCommanded, 0.1))
        * DrivetrainSubsystem.kMaxSpeed;

    // Get the y speed or sideways/strafe speed
    final var yPower =
      RobotContainer.m_yspeedLimiter.calculate(MathUtil.applyDeadband(yPowerCommanded, 0.1))
        * DrivetrainSubsystem.kMaxSpeed;

    // Get the rate of angular rotation
    final var rot =
    //must be positive to read accuate joystick yaw
      RobotContainer.m_rotLimiter.calculate(MathUtil.applyDeadband(rotCommanded, 0.2))
        * this.kMaxAngularSpeed;

    this.drive(xPower, yPower, rot, true);
    // This method will be called once per scheduler run
    updateOdometry();
  }

  public void recalibrateGyro() {
    System.out.println(m_gyro.getRotation2d());
    m_gyro.calibrate();
    System.out.println(m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).   -1.0 ... +1.0
   * @param ySpeed Speed of the robot in the y direction (sideways).  -1.0 ... +1.0
   * @param rot Angular rate of the robot.                            -1.0 ... +1.0
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= kMaxSpeed;
    ySpeed *= kMaxSpeed;
    rot *= kMaxAngularSpeed;
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  /** Get pose from odometry field **/
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return m_kinematics; 
  }

  /** zeroes drivetrain odometry **/
  public void zeroOdometry() {
    resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/subsystems/DriveSubsystem.java
   * 
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        },
        pose);
  }

  /** Sets the swerve ModuleStates.
   * @param desiredStates The desired SwerveModule states as a ChassisSpeeds object
   */
  private void setDesiredStates(ChassisSpeeds cs) {
    // System.out.println("vX: " + Math.round(cs.vxMetersPerSecond*100.0)/100.0 + "  vY: " + Math.round(cs.vyMetersPerSecond));
    SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(cs);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, 4); //TODO: Constants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  } 

public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState());
    return chassisSpeeds;
  }

  /** Follows PathPlanner trajectory. Used in auto sequences.
   * <p> https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage#ppswervecontrollercommand
   * 
   * @param traj Trajectory. Need to first load trajectory from PathPlanner traj. That trajectory goes in here.
   * @param isFirstPath Whether this is the first path in the sequence. If this is TRUE, the odometry will be zeroed at the start of the command sequence.
  **/
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, Boolean isFirstPath) {
    InstantCommand ic = new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(isFirstPath){
          this.zeroOdometry();
        }
      });

    PPSwerveControllerCommand c = new PPSwerveControllerCommand(
      traj, 
      this::getPose, 
      new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D), 
      new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D), 
      new PIDController(ROTATION_P, ROTATION_I, ROTATION_D), 
      this::setDesiredStates, 
      isFirstPath, 
      this
    );

    return new SequentialCommandGroup(ic, c);
  }

  /** Follows PathPlanner trajectory. Used in auto sequences.
   * <p> https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage#ppswervecontrollercommand
   * 
   * @param traj Trajectory. Need to first load trajectory from PathPlanner traj. That trajectory goes in here.
  **/
  public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
    PPSwerveControllerCommand c = new PPSwerveControllerCommand(
      traj, 
      this::getPose, 
      new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D), 
      new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D), 
      new PIDController(ROTATION_P, ROTATION_I, ROTATION_D), 
      this::setDesiredStates, 
      this
    );

    return c;
  }

  public NavX getGyroscope() {
    return m_gyro; 
  }

  /**Sets the swerve ModuleStates.
   * @param desiredStates The desired SwerveModule states. Array of `SwerveModuleState[]`
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, this.kMaxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }
  
  /** Displays all 4 module positions + robot pose (forward/back) in SmartDashboard. 
   * </p> For debugging
   */
  public void putDTSToSmartDashboard() {
    // SmartDashboard.putNumber("Front Left Pos", m_frontLeft.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("Front Right Pos", m_frontRight.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("Back Left Pos", m_backLeft.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("Back Right Pos", m_backRight.m_driveEncoder.getPosition()); 

    SmartDashboard.putNumber("Drive Pose X", getPose().getTranslation().getX());
    SmartDashboard.putNumber("Drive Pose Y", getPose().getTranslation().getY());
  }

  
}
