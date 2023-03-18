// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pose;
import frc.robot.RobotContainer;

public class SetTargetPoseCommand extends CommandBase {

  Pose m_targetPose;

  /** Creates a new SetTargetPoseCommand. */
  public SetTargetPoseCommand(Pose targetPose) {
    m_targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.setTargetPose(m_targetPose);
    // System.out.println("SetTargetPoseCommand:initialize: " + m_targetPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Command should just do initialize and then it is complete
  
  @Override
  public boolean isFinished() {
    return true;
  }
}
