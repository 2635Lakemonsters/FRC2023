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
  // Now that we do not care about state transitions, next command can start immediately
  // in parallel to this command starting.  
  // CAUTION.  If we are scoring in autonomous immediately after the arm moves into position,
  //           we may want to have a bit of a delay.  otherwise, the gripper may open after the 
  //           arm moves before the arm actually finishes transitioning with pneumatics
  // Note that the delay on the extend is intended to give extra room at the start between the 
  //           gripper and the battery for the first autonomous score.
  double m_delay_retract = 0.; // 1.0 / 2; // seconds
  double m_delay_extend = 0.5; // .5 / 3;
  boolean is_extending; 

  /** Creates a new IntakePneumaticCommand. */
  public ArmPneumaticCommand(ArmPneumaticSubsystem armPneumaticSubsystem, boolean bExtend) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armPneumaticSubsystem = armPneumaticSubsystem;
    this.bExtend = bExtend;
    
    addRequirements(m_armPneumaticSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("APC: " + bExtend);
    if(bExtend) { // if extended then retract
      is_extending = false;
      m_armPneumaticSubsystem.armExtend();
    } else { // if retracted then extend
      is_extending = true; 
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
    if (is_extending) {
      return m_endTime - m_startTime > m_delay_extend;
    } else {
      return m_endTime - m_startTime > m_delay_retract;
    }
  }
}
