// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SwerveDriveBalanceCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.drivers.NavX;

public class DrivetrainSubsystem extends SubsystemBase {

    // public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxSpeed = 0.4;
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  
    public final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    public final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    public final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    public final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    public final SwerveModule m_frontLeft = new SwerveModule(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER, 
                                                              Constants.FRONT_LEFT_ANGLE_OFFSET_COMPETITION,
                                                              Constants.DRIVETRAIN_FRONT_LEFT_ANALOGID);
    public final SwerveModule m_frontRight = new SwerveModule(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER, 
                                                              Constants.FRONT_RIGHT_ANGLE_OFFSET_COMPETITION,
                                                              Constants.DRIVETRAIN_FRONT_RIGHT_ANALOGID);
    public final SwerveModule m_backLeft = new SwerveModule(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER, 
                                                              Constants.BACK_LEFT_ANGLE_OFFSET_COMPETITION,
                                                              Constants.DRIVETRAIN_BACK_LEFT_ANALOGID);
    public final SwerveModule m_backRight = new SwerveModule(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER, 
                                                              Constants.BACK_RIGHT_ANGLE_OFFSET_COMPETITION,
                                                              Constants.DRIVETRAIN_BACK_RIGHT_ANALOGID);
  
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

  /** Creates a new DrivetrianSubsystem. */
  public DrivetrainSubsystem() {
    m_gyro.calibrate();
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
    // Get the x speed
    final var xPower =
      RobotContainer.m_xspeedLimiter.calculate(MathUtil.applyDeadband(xPowerCommanded, 0.02))
        * DrivetrainSubsystem.kMaxSpeed;

    // Get the y speed or sideways/strafe speed
    final var yPower =
      RobotContainer.m_yspeedLimiter.calculate(MathUtil.applyDeadband(yPowerCommanded, 0.02))
        * DrivetrainSubsystem.kMaxSpeed;

    // Get the rate of angular rotation
    final var rot =
    //must be positive to read accuate joystick yaw
      RobotContainer.m_rotLimiter.calculate(MathUtil.applyDeadband(rotCommanded, 0.02))
        * DrivetrainSubsystem.kMaxAngularSpeed;

    RobotContainer.m_drivetrainSubsystem.drive(xPower, yPower, rot, true);
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
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
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

  
}
