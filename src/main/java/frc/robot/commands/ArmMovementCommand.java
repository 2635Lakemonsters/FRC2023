// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmMotorSubsystem;

/**
  * Moves upper arm to target position
 */
public class ArmMovementCommand extends CommandBase {

  private ArmMotorSubsystem m_armMotorSubsystem;
  private double m_targetPose;
  private RobotContainer.Poser m_getPose = null;

  /** Creates a new ArmMoveCommand. */
  public ArmMovementCommand(ArmMotorSubsystem armMotorSubsystem, RobotContainer.Poser getPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armMotorSubsystem = armMotorSubsystem;
    m_getPose = getPose;

    addRequirements(m_armMotorSubsystem);
  }

  /** Creates a new ArmMoveCommand. */
  public ArmMovementCommand(ArmMotorSubsystem armMotorSubsystem, double targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armMotorSubsystem = armMotorSubsystem;
    m_targetPose = targetPose;

    addRequirements(m_armMotorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_getPose != null)
      m_targetPose = m_getPose.execute().targetTheta;
    System.out.println("AMC: " + m_targetPose);
    m_armMotorSubsystem.setPose(m_targetPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean areWeThereYet = m_armMotorSubsystem.areWeThereYet();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
