// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.networktables.NetworkTableEntry;

import frc.robot.Constants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  public final RelativeEncoder m_driveEncoder;
  private final AnalogInput m_turningEncoder;

  private double turningMotorOffset;

  private final PIDController m_drivePIDController = new PIDController(0.005, 0, 0.);
  private final PIDController m_turningPIDController = new PIDController(Constants.kPModuleTurningController, 0, 0.0001);

  public NetworkTableEntry t_turningEncoder;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int analogEncoderPort,
      double turningMotorOffset
      ) {

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    this.turningMotorOffset = turningMotorOffset;

  
    m_turningEncoder = new AnalogInput(analogEncoderPort);

    m_driveEncoder = m_driveMotor.getEncoder();
    
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(Constants.kDriveEncoderDistancePerPulse);
    m_driveEncoder.setVelocityConversionFactor(Constants.kDriveEncoderDistancePerPulse/60.0);


    // Set whether drive encoder should be reversed or not
    // m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.

    // Set whether turning encoder should be reversed or not
    // m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void updateSwerveTable() {
    t_turningEncoder.setDouble(Math.toRadians(m_turningMotor.getEncoder().getPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(getTurningEncoderRadians()));
  }

  private double getTurningEncoderRadians(){
    double angle = (1.0 - m_turningEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI + turningMotorOffset;
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
        angle += 2.0 * Math.PI;
    }
    return angle;
    }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getTurningEncoderRadians()));
  }

  private static int loopCtr = 0;
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderRadians()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = //state.speedMetersPerSecond;
      m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedForward = state.speedMetersPerSecond / DrivetrainSubsystem.kMaxSpeed;

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(getTurningEncoderRadians(), state.angle.getRadians());

    loopCtr++;
    if ((loopCtr % 50 == 0) && (m_driveMotor.getDeviceId() == 8))
    {
      
      System.out.println(
        "Spd: " + Math.round(state.speedMetersPerSecond * 100.) / 100. + 
        "  getV(): " + Math.round(m_driveEncoder.getVelocity() * 100.) / 100. + 
        "  DO: "+ Math.round(driveOutput * 100.) / 100.
      );
    }

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput + driveFeedForward);
    m_turningMotor.set(turnOutput);
  }
}
