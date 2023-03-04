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
public class FPlus2FMinusCommand extends SequentialCommandGroup {
  /** Creates a new TopScoringArmMovementCommand. */
  public FPlus2FMinusCommand(ArmPneumaticSubsystem armPneumaticSubsystem, ArmMotorSubsystem armMotorSubsystem, RobotContainer.Poser p) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new ArmPneumaticCommand(armPneumaticSubsystem, false),
      new WaitCommand(0.5), //required from pickup to top score
      new ArmMovementCommand(armMotorSubsystem, Constants.Vplus + 30),
      new ArmPneumaticCommand(armPneumaticSubsystem, true),
      new ArmMovementCommand(armMotorSubsystem, Constants.Hminus - 30),
      new WaitCommand(0.8),
      new ArmMovementCommand(armMotorSubsystem, p) // target pose
    );
  }
}