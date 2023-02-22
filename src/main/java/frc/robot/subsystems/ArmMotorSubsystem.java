// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmMotorSubsystem extends SubsystemBase {

  public TalonFX armMotor = new TalonFX(Constants.TALON_CHANNEL);
  private static final double kFilterArm = 0.1;
  private static double armPOFiltered = 0;
  private static long loopCtr = 0;
  
  /** Creates a new ArmMotorSubsystem. */
  public ArmMotorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double calculatePower(double targetPose) { // target encoder pose
    double currentPose = RobotContainer.encoder.get(); // Get current encoder pose 
    double delta = targetPose - currentPose;

    final double gain = 2.0;
    final double range = 0.15;
    double motorPower = gain * delta;
    motorPower = Math.min(Math.max(motorPower, -range), range);

    return motorPower;
  }
  
  public void setMotorPower(double motorPower) {
    /*Set Voltage in this method*/
    armMotor.set(ControlMode.PercentOutput, motorPower);
  }

  public void setPose(double pose) {
    loopCtr++;

    double alpha = ArmPneumaticSubsystem.getIsExtended() ? 116.3 : 80.3;
    pose = RobotContainer.encoder.getDistance() - RobotContainer.m_armEncoderOffset;
    double fPO = (pose - alpha + 90.0);

    final double gain = 0.2;
    double motorPower = gain * Math.sin(Math.toRadians(fPO));

    // armPOFiltered = kFilterArm * motorPower + (1.0 - kFilterArm) * armPOFiltered;
    // System.out.println(armPOFiltered);
    if (loopCtr%50 == 0) {
      System.out.println(pose);
      System.out.println("Motor Power = " + motorPower);
      System.out.println("Angle = " + fPO);
    }

    // armMotor.set(ControlMode.PercentOutput, -motorPower);
  }
 }
