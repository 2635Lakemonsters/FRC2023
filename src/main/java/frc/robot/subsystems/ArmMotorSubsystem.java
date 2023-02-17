// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// @Darren From our discussion on Thursday
// Make a new subsystem to manage the overall arm.
// * constructor parameters will include
//   * ArmMotorSubsystem
//   * ArmPneumaticSubsystem
//   * ArmEncoderSensor or whatever
// Calculate the appropriate feed forward to set in the ArmMotorSubsystem to take care of gravity.
// Continuously feed that to ArmMotorSubsystem...  Note that in state transitions for pneumatics, we may have 
// some bounce as the zero for the feed forward fluctuates, but that shouldn't be too bad, I hope... feed back 
// should take care of it?
//
// the ArmMotorSubsystem should be responsible for applying whatever feed forward is needed.
// the ArmSubsystem should be responsible for telling the ArmMotorSubsystem what feed forward it should be using
//   since it knows about the state of the pneumatics.
// the ArmSubsystem should also have the encoder so it knows how to calculate the feed forward from the combination of the 
//   arm encoder and state of the pneumatic system
// 
// ====
// Ramp Command
// * pushing down the ramp might be posible with only the motor, will need finesse
// * setting the arm in the right position to push down when pneumatics go forward might be good.
//   * doing this will need a schedule of as you ascend the ramp, will need to move the arm.
//   * potentially could get the initial position for this movement from distance of april tag on the oposite side of ramp 
// .   if we are facing the april tags, Otherwise, don't know yet what to do...
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
