// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMovementCommand;
import frc.robot.commands.ArmPneumaticCommand;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BPlus2FMinusCommand extends SequentialCommandGroup {
  public BPlus2FMinusCommand(ArmPneumaticSubsystem armPneumaticSubsystem, ArmMotorSubsystem armMotorSubsystem, RobotContainer.Poser p) {

    addCommands(
      new ArmPneumaticCommand(armPneumaticSubsystem, true),
      new ArmMovementCommand(armMotorSubsystem, p) // target pose
    );
  }
}