// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import frc.robot.subsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class AlignGripperToObjectCommand extends CommandBase {
  DrivetrainSubsystem m_drivetrainSubsystem;
  ObjectTrackerSubsystem m_objectTrackerSubsystem;
  ArmPneumaticSubsystem m_armPneumaticSubsystem;
  ClawPneumaticSubsystem m_clawPneumaticSubsystem;
  
  /** Creates a new AlignGripperToObjectCommand. 
   * Once arm is in position, this command centers/aligns the bot for the gripper to close around
   * 
   * 1) make sure gripper is open at start (initialize) use the armpneumaticcommand OR armpneumatic subsystem
   * 2) make sure arm is in right configuration
   * 3) alignment (execute) - call drivetrainSubsystem methods? and/or move the arm to align to the object
   * 4) Align claw to the object using armpneumaticsubsystem and clawpneumaticsubsystem 
  */
  public AlignGripperToObjectCommand(DrivetrainSubsystem ds, ObjectTrackerSubsystem ots) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_clawPneumaticSubsystem.grabberOpen();
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
    return false;
  }
}
