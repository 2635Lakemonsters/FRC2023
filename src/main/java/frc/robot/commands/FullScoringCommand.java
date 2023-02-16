// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import frc.robot.subsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullScoringCommand extends SequentialCommandGroup {
  /** Creates a new DriveAndPrepareIntakeCommand. */
  public FullScoringCommand(DrivetrainSubsystem drivetrainSubsystem, ObjectTrackerSubsystem objectTrackerSubsystemChassis, ArmPneumaticSubsystem armPneumaticSubsystem, int buttonNumber, ClawPneumaticSubsystem clawPneumaticSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new GoScoreCommand(drivetrainSubsystem, objectTrackerSubsystemChassis, buttonNumber),
        new ControlScoringCommands(armPneumaticSubsystem, buttonNumber, false)
      ),
      new ClawPneumaticCommand(clawPneumaticSubsystem),
      new ControlScoringCommands(armPneumaticSubsystem, 20, true)
    );
  }
}
