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

  // feed forward for calculating power will require knowledge of state of the pneumatic controller.
  // you will need to know the arm encoder 0 angle orientation in both pneumatic piston states.
  // pick an orientation reletive to gravity to decide on what should be 0
  // use that angle to calcualte a torque for feed forward so that the arm does not fall down.
  // depending upon how you define what 0 is, your feed forward will be a different trigonometric function.
  // you define the coordinate system so you can either make your life easy or difficult by your choice.
  //
  // will need feedback to put the arm in the actual desired state.
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
