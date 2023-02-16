// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmMotorSubsystem extends SubsystemBase {
  /** Creates a new ArmMotorSubsystem. */
  public ArmMotorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static double calculatePower(double targetPose){
    double currentPose = /*Get current pose */ 0;
    double delta = targetPose-currentPose;

    final double gain = 4.0;
    final double range = 2.0;
    double lockPose = gain * delta;

    lockPose = Math.min(Math.max(lockPose, -range), range);

    return lockPose;
  }
  
  public void setMotorPower(double targetPose){
    /*Set Voltage in this method*/
    double power = calculatePower(targetPose);
  }
}
