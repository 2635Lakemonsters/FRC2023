// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FromTopToHomeCommand extends SequentialCommandGroup {
  /** Creates a new TopScoringArmMovementCommand. */
  public FromTopToHomeCommand(ArmPneumaticSubsystem armPneumaticSubsystem, ArmMotorSubsystem armMotorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // TODO: figure out mid state angle
    
    addCommands(
      new ParallelCommandGroup(
        new ArmPneumaticCommand(armPneumaticSubsystem),
        new ArmMovementCommand(armMotorSubsystem, 20)
      ),
      new ArmMovementCommand(armMotorSubsystem, Constants.HOME_ARM_ANGLE)
    );
  }
}
