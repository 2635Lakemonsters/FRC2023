// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;

public class ArmPoseSelectorCommand extends CommandBase {

  private ArmPneumaticSubsystem m_armPneumaticSubsystem;
  private ArmMotorSubsystem m_armMotorSubsystem;
  private int m_buttonNum;
  private boolean m_returnToDock;
  // TopScoringArmMovementCommand m_topScoringArmMovementCommand;
  // MidScoringArmMovementCommand m_midScoringArmMovementCommand;
  // FromTopToHomeCommand m_fromTopReturnHomeCommand;
  // FromMidToHomeCommand m_fromMidReturnHomeCommand;

  /** Creates a new ControlScoringCommands. */
  public ArmPoseSelectorCommand(ArmPneumaticSubsystem armPneumaticSubsystem, ArmMotorSubsystem armMotorSubsystem, int buttonNum, boolean returnToDock) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_buttonNum = buttonNum;
    m_returnToDock = returnToDock;
    m_armPneumaticSubsystem = armPneumaticSubsystem;
    m_armMotorSubsystem = armMotorSubsystem;

    // m_topScoringArmMovementCommand = new TopScoringArmMovementCommand(m_armPneumaticSubsystem, m_armMotorSubsystem);
    // m_midScoringArmMovementCommand = new MidScoringArmMovementCommand(m_armPneumaticSubsystem, m_armMotorSubsystem);
    // m_fromTopReturnHomeCommand = new FromTopToHomeCommand(m_armPneumaticSubsystem, m_armMotorSubsystem);
    // m_fromMidReturnHomeCommand = new FromMidToHomeCommand(m_armPneumaticSubsystem, m_armMotorSubsystem);
   
    addRequirements(armPneumaticSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_returnToDock) {
      if (m_buttonNum == 7 || m_buttonNum == 8) {
        // top arm position command
        // RobotContainer.select();
      } else if (m_buttonNum == 9 || m_buttonNum == 10) {
        // middle arm position command
        // m_midScoringArmMovementCommand.execute();
      } 
    } else if (m_returnToDock){
      if (m_buttonNum == 5 || m_buttonNum == 6 || m_buttonNum == 0) {
        // going back to the home pose from top scoring
        // m_fromTopReturnHomeCommand.execute();
      } else if (m_buttonNum == 3 || m_buttonNum == 4|| m_buttonNum == 180) {
        // going back to the home pose from mid scoring
        // m_fromMidReturnHomeCommand.execute();
      }
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
