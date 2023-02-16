// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmMotorSubsystem extends SubsystemBase {

  private TalonFX armMotor = new TalonFX(0); //figure out the devie number
  private TalonFXControlMode controlMode;

  /** Creates a new ArmMotorSubsystem. */
  public ArmMotorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double calculatePower(double targetPose) { // target encoder pose
    double currentPose = 0; // Get current encoder pose 
    double delta = targetPose - currentPose;

    final double gain = 4.0;
    final double range = 2.0;
    double motorPower = gain * delta;
    motorPower = Math.min(Math.max(motorPower, -range), range);

    return motorPower;
  }
  
  public void setMotorPower(double motorPower) {
    /*Set Voltage in this method*/
    armMotor.set(controlMode, motorPower);
  }
}
