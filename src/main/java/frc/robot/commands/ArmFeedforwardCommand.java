// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmFeedforwardSubsystem;
import frc.robot.subsystems.ArmMotorSubsystem;

public class ArmFeedforwardCommand extends CommandBase {

  private ArmMotorSubsystem m_armMotorSubsystem;
  private double feedforward;

  /** Creates a new ArmFeedforwardCommand. */
  public ArmFeedforwardCommand(ArmMotorSubsystem armMotorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armMotorSubsystem = armMotorSubsystem;

    addRequirements(m_armMotorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feedforward = RobotContainer.encoder.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armMotorSubsystem.setPose(feedforward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armMotorSubsystem.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
