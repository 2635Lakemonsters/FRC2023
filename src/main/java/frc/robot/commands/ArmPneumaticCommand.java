// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import edu.wpi.first.wpilibj.Timer;

/**
  * Moves lower arm between retracted and extended configurations
 */
public class ArmPneumaticCommand extends CommandBase {
  ArmPneumaticSubsystem m_armPneumaticSubsystem;
  public boolean bExtend;

  Timer m_ticktock = new Timer(); 
  double m_startTime;
  double m_endTime;
  double m_delay = 1.5; // seconds

  /** Creates a new IntakePneumaticCommand. */
  public ArmPneumaticCommand(ArmPneumaticSubsystem armPneumaticSubsystem, boolean bExtend) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armPneumaticSubsystem = armPneumaticSubsystem;
    this.bExtend = bExtend;
    
    addRequirements(m_armPneumaticSubsystem);
  }

  // TODO: Start a timer.  Don't finish until we believe arm has finished moving.
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("APC: " + bExtend);
    if(bExtend) { // if extended then retract
      m_armPneumaticSubsystem.armExtend();
    } else { // if retracted then extend
      m_armPneumaticSubsystem.armRetract();
    }

    m_ticktock.start(); 
    m_startTime = m_ticktock.get(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_endTime = m_ticktock.get(); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_endTime - m_startTime > m_delay;
  }
}
