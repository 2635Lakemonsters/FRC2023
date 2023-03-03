// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SwerveDriveCommand extends CommandBase {

  private static DrivetrainSubsystem m_drivetrainSubsystem;

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     System.out.println("SDC.initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set the x power commanded
    DrivetrainSubsystem.setXPowerCommanded(RobotContainer.rightJoystick.getY());
    DrivetrainSubsystem.setYPowerCommanded(RobotContainer.rightJoystick.getX());
    // DrivetrainSubsystem.setRotCommanded(RobotContainer.leftJoystick.getX()); // uncomment this to use left joystick for rotation
    DrivetrainSubsystem.setRotCommanded(-RobotContainer.rightJoystick.getTwist()); // uncomment this to use right joystick twist
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SDC.end()");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
