// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmMotorSubsystem;

public class ControlScoringCommands extends CommandBase {
  /** Creates a new ControlScoringCommands. */
  private int buttonNum;
  private boolean returnToDock;
  public ControlScoringCommands(int buttonNum, boolean returnToDock) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.buttonNum = buttonNum;
    this.returnToDock = returnToDock;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(buttonNum == 5 || buttonNum == 6 || buttonNum == 0){
      //top arm position command
      new SequentialCommandGroup(
        ArmMotorSubsystem.calculatePower(20),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(),
        ArmMotorSubsystem.calculatePower(20),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(),
        ArmMotorSubsystem.calculatePower(20),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand()
      );
    }
    else if(buttonNum == 3 || buttonNum == 4|| buttonNum == 180){
      //middle arm position command
      new SequentialCommandGroup(
        ArmMotorSubsystem.calculatePower(20),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(),
        ArmMotorSubsystem.calculatePower(20),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand(),
        ArmMotorSubsystem.calculatePower(20),
        ArmMotorSubsystem.setMotorPower(20),
        new ArmPneumaticCommand()
      );
    }
    else if(returnToDock && buttonNum == 20){
      new SequentialCommandGroup(
        new ArmPneumaticCommand(),
        ArmMotorSubsystem.calculatePower(),
        ArmMotorSubsystem.setMotorPower(),
        new ArmPneumaticCommand(),
        ArmMotorSubsystem.calculatePower(),
        ArmMotorSubsystem.setMotorPower(),
        new ArmPneumaticCommand(),
        ArmMotorSubsystem.calculatePower(),
        ArmMotorSubsystem.setMotorPower()
      );      
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
