// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;

public class ArmPoseControlCommand extends CommandBase {

  private ArmPneumaticSubsystem m_armPneumaticSubsystem;
  private ArmMotorSubsystem m_armMotorSubsystem;
  private int m_buttonNum;
  private boolean m_returnToDock;
  TopScoringArmMovementCommand m_topScoringArmMovementCommand = new TopScoringArmMovementCommand(m_armPneumaticSubsystem, m_armMotorSubsystem);
  MidScoringArmMovementCommand m_midScoringArmMovementCommand = new MidScoringArmMovementCommand(m_armPneumaticSubsystem, m_armMotorSubsystem);
  DockingArmMovementCommand m_dockingArmMovementCommand = new DockingArmMovementCommand(m_armPneumaticSubsystem, m_armMotorSubsystem);

  /** Creates a new ControlScoringCommands. */
  public ArmPoseControlCommand(ArmPneumaticSubsystem armPneumaticSubsystem, ArmMotorSubsystem armMotorSubsystem, int buttonNum, boolean returnToDock) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_buttonNum = buttonNum;
    m_returnToDock = returnToDock;
    m_armPneumaticSubsystem = armPneumaticSubsystem;
    m_armMotorSubsystem = armMotorSubsystem;
   
    addRequirements(armPneumaticSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  //
  // You could possibly refine this and define other poses, but transitioning between these 7 states sequentially in either 
  // way should keep us from violating the rules.  Please review / validate that this is the case.
  // Pose 0 - Pneumatic 0 - starting pose
  // Pose 1 - Pneumatic 0 - pickup from human next to april command
  // Pose 2 - Pneumatic 1 - same as pose 1, but transitioned pneumatics
  // Pose 3 - Pneumatic 1 - top score pose
  // Pose 4 - Pneumatic 0 - same as pose 3, but popped back
  // Pose 5 - Pneumatic 0 - arm transitions to proper mid score pose, still in back position
  // Pose 6 - Pneumatic 1 - pop forward to mid score position
  // Pose 7 - Pneumatic 1 - lower arm to lower score / pickup item position
  //
  // Ramp ascent states.  Can go direct to this position from any one of 5,6,7 
  // Pose x.1 - Pneumatic 1 - arm forward as we approach ramp, arm rotates down and pushes ramp down as we get closer to the ramp.
  //
  // Experiment to figure out a different set of states to ascend the ramp.
  // * should we use april tag to know our distance and push down vs distance from april tag?
  // * do we need an auxiliary arm specifically for ascending pointing in the oposite direction? (Yikes, more complexity... but maybe needed)
  @Override
  public void execute() {
    if(m_buttonNum == 5 || m_buttonNum == 6 || m_buttonNum == 0){
      // top arm position command
      m_topScoringArmMovementCommand.execute();
    } else if(m_buttonNum == 3 || m_buttonNum == 4|| m_buttonNum == 180){
      // middle arm position command
      m_midScoringArmMovementCommand.execute();
    } else if(m_returnToDock && m_buttonNum == Constants.DOCKING_BUTTON_NUMBER){
      // going back to the docked pose
      m_dockingArmMovementCommand.execute();
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
