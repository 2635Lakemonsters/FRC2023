// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ARM_TRANSITION;
import frc.robot.Pose;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Poser;
import frc.robot.util;
import frc.robot.commands.Transitions.BMinus2BMinusCommand;
import frc.robot.commands.Transitions.BMinus2BPlusCommand;
import frc.robot.commands.Transitions.BMinus2FMinusCommand;
import frc.robot.commands.Transitions.BMinus2FPlusCommand;
import frc.robot.commands.Transitions.BPlus2BMinusCommand;
import frc.robot.commands.Transitions.BPlus2BPlusCommand;
import frc.robot.commands.Transitions.BPlus2FMinusCommand;
import frc.robot.commands.Transitions.BPlus2FPlusCommand;
import frc.robot.commands.Transitions.FMinus2BMinusCommand;
import frc.robot.commands.Transitions.FMinus2BPlusCommand;
import frc.robot.commands.Transitions.FMinus2FMinusCommand;
import frc.robot.commands.Transitions.FMinus2FPlusCommand;
import frc.robot.commands.Transitions.FPlus2BMinusCommand;
import frc.robot.commands.Transitions.FPlus2BPlusCommand;
import frc.robot.commands.Transitions.FPlus2FMinusCommand;
import frc.robot.commands.Transitions.FPlus2FPlusCommand;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;


public class MoveArmToPoseCommand extends SequentialCommandGroup {
  /** Creates a new MoveArmToPoseCommand. */
  ArmPneumaticSubsystem m_armPneumaticSubsystem;
  ArmMotorSubsystem m_armMotorSubsystem;
  Poser m_getPose;
  
  public ARM_TRANSITION select() {
    Pose pose = RobotContainer.getTargetPose();
    ARM_TRANSITION trans = util.getTransition(pose.targetExtend, pose.targetTheta);
    // System.out.println("select: " + trans);
    return trans;
  }



  public MoveArmToPoseCommand(ArmPneumaticSubsystem armPneumaticSubsystem, ArmMotorSubsystem armMotorSubsystem, Poser p) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armPneumaticSubsystem = armPneumaticSubsystem;
    m_armMotorSubsystem = armMotorSubsystem; 
    m_getPose = p;
    // System.out.println("Poser: " + m_getPose);

    
    addCommands(
      new SelectCommand(
        // Maps selector values to commands
        Map.ofEntries(
          Map.entry(ARM_TRANSITION.BMinus2BMinus, new BMinus2BMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.BMinus2BPlus, new BMinus2BPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.BMinus2FMinus, new BMinus2FMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.BMinus2FPlus, new BMinus2FPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.BPlus2BMinus, new BPlus2BMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.BPlus2BPlus, new BPlus2BPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.BPlus2FMinus, new BPlus2FMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.BPlus2FPlus, new BPlus2FPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.FMinus2BMinus, new FMinus2BMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.FMinus2BPlus, new FMinus2BPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.FMinus2FMinus, new FMinus2FMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.FMinus2FPlus, new FMinus2FPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.FPlus2BMinus, new FPlus2BMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.FPlus2BPlus, new FPlus2BPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.FPlus2FMinus, new FPlus2FMinusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose)),
          Map.entry(ARM_TRANSITION.FPlus2FPlus, new FPlus2FPlusCommand(m_armPneumaticSubsystem, m_armMotorSubsystem, m_getPose))),
        this::select)
    );
  }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {}

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
