// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class AutoClawCloseCommand extends CommandBase {

  ObjectTrackerSubsystem m_objectTrackerSubsystem;
  ClawPneumaticCommand m_clawPneumaticCommand;
  boolean isObjectCloseEnough;

  /** Creates a new AutoClawCloseCommand. */
  public AutoClawCloseCommand(ObjectTrackerSubsystem objectTrackerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_objectTrackerSubsystem = objectTrackerSubsystem;

    addRequirements(m_objectTrackerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isObjectCloseEnough = m_objectTrackerSubsystem.isGripperCloseEnough();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (!isObjectCloseEnough) {
      isObjectCloseEnough = m_objectTrackerSubsystem.isGripperCloseEnough();
    }
    if (isObjectCloseEnough) {
      m_clawPneumaticCommand.execute();
    }
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
