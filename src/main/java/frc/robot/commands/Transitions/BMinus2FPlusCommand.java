// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Transitions;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BMinus2FPlusCommand extends SequentialCommandGroup {
  /** Creates a new TopScoringArmMovementCommand. */
  public BMinus2FPlusCommand(ArmPneumaticSubsystem armPneumaticSubsystem, ArmMotorSubsystem armMotorSubsystem, RobotContainer.Poser p) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands( // TODO: WRONG BUT WILL NOT USE
      // new ArmPneumaticCommand(armPneumaticSubsystem, true),
      // new ArmMovementCommand(armMotorSubsystem, Constants.Vplus),
      // new ArmPneumaticCommand(armPneumaticSubsystem, false),
      // new ArmMovementCommand(armMotorSubsystem, Constants.Hplus),
      // new ArmPneumaticCommand(armPneumaticSubsystem, true),
      // new WaitCommand(0.5),
      // new ArmMovementCommand(armMotorSubsystem, p), // target pose
      new PrintCommand("this is wrong do not use")
    );
  }
}