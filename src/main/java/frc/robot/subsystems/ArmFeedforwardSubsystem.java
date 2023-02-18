// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmFeedforwardSubsystem extends SubsystemBase {

  private ArmMotorSubsystem m_armMotorSubsystem;
  private ArmPneumaticSubsystem m_armPneumaticSubsystem;

  /** Creates a new ArmFeedforwardSubsystem. */
  public ArmFeedforwardSubsystem(ArmMotorSubsystem armMotorSubsystem, ArmPneumaticSubsystem armPneumaticSubsystem) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double calculateFeedforward() {
    // calculates the feedforward based on the angle of the arm
    // get current encoder pose
    // figure out the arm pnuematic pose
    // get the change in the angle if the pnuematic is activated or if its not
    // return the velocity necessary to keep it where its at
    return 0.0;
  }
}
