// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;

public class ControlScoringCommands extends CommandBase {

  private ArmPneumaticSubsystem m_armPneumaticSubsystem;
  private int m_buttonNum;
  private boolean m_returnToDock;

  /** Creates a new ControlScoringCommands. */
  public ControlScoringCommands(ArmPneumaticSubsystem armPneumaticSubsystem, int buttonNum, boolean returnToDock) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_buttonNum = buttonNum;
    m_returnToDock = returnToDock;
    m_armPneumaticSubsystem = armPneumaticSubsystem;
   
    addRequirements(armPneumaticSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_buttonNum == 5 || m_buttonNum == 6 || m_buttonNum == 0){
      // top arm position command
      new SequentialCommandGroup(
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(m_armPneumaticSubsystem),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(m_armPneumaticSubsystem),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(m_armPneumaticSubsystem)
      );
    } else if(m_buttonNum == 3 || m_buttonNum == 4|| m_buttonNum == 180){
      // middle arm position command
      new SequentialCommandGroup(
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(m_armPneumaticSubsystem),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(m_armPneumaticSubsystem),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(m_armPneumaticSubsystem)
      );
    } else if(m_returnToDock && m_buttonNum == 20){
      // going back to the docked pose
      new SequentialCommandGroup(
        new ArmPneumaticCommand(m_armPneumaticSubsystem),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(m_armPneumaticSubsystem),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(m_armPneumaticSubsystem),
        ArmMotorSubsystem.setMotorPower(20)
      );      
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
