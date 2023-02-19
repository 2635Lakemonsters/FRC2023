// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmFeedforwardSubsystem extends SubsystemBase {

  private ArmMotorSubsystem m_armMotorSubsystem;
  private ArmPneumaticSubsystem m_armPneumaticSubsystem;

  /** Creates a new ArmFeedforwardSubsystem. */
  public ArmFeedforwardSubsystem(ArmMotorSubsystem armMotorSubsystem, ArmPneumaticSubsystem armPneumaticSubsystem) {
    m_armMotorSubsystem = armMotorSubsystem;
    m_armPneumaticSubsystem = armPneumaticSubsystem;
  }

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
    double feedforward = m_armMotorSubsystem.calculatePower(RobotContainer.encoder.get());
    return feedforward;
  }

  public void setFeedforward(double armfeedforward) {
    // armMotor.set(ControlMode.Position, RobotContainer.encoder.get(), DemandType.ArbitraryFeedForward, Math.cos((RobotContainer.encoder.get()) / 4096 * 360 * 0.165));
  }
}
