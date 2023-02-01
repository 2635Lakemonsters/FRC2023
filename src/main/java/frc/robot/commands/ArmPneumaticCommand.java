// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPneumaticSubsystem;

public class ArmPneumaticCommand extends CommandBase {
  ArmPneumaticSubsystem m_armPneumaticSubsystem;
  public boolean isExtended;

  /** Creates a new IntakePneumaticCommand. */
  public ArmPneumaticCommand(ArmPneumaticSubsystem armPneumaticSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armPneumaticSubsystem = armPneumaticSubsystem;
    addRequirements(m_armPneumaticSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isExtended = m_armPneumaticSubsystem.isExtended();
    if(isExtended) { // if extended then retract
      m_armPneumaticSubsystem.armRetract();
    } else if(!isExtended) { // if retracted then extend
      m_armPneumaticSubsystem.armExtend();
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
