// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BMinus2BMinusCommand extends SequentialCommandGroup {
  /** Creates a new TopScoringArmMovementCommand. */
  public BMinus2BMinusCommand(ArmPneumaticSubsystem armPneumaticSubsystem, ArmMotorSubsystem armMotorSubsystem, int targetpose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new ArmMovementCommand(armMotorSubsystem, targetpose) // target pose
    );
  }
}