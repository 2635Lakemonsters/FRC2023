// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawPneumaticSubsystem;

public class ClawPneumaticCommand extends CommandBase {
  ClawPneumaticSubsystem m_clawPneumaticSubsystem;
  public boolean isClosed;

  /** Creates a new IntakePneumaticCommand. */
  public ClawPneumaticCommand(ClawPneumaticSubsystem clawPneumaticSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_clawPneumaticSubsystem = clawPneumaticSubsystem;
    addRequirements(m_clawPneumaticSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isClosed = m_clawPneumaticSubsystem.isClosed();
    if(isClosed) { //if closed then open
      m_clawPneumaticSubsystem.grabberOpen();
    } else if(!isClosed) { // if open then close
      m_clawPneumaticSubsystem.grabberClose();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}