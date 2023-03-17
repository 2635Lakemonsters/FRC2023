// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

/** SwerveDriveBalanceJerky
 * </p> Jerk move step by step until balance tips. Then move one step back.  */
public class SwerveDriveBalanceJerky extends CommandBase {
  DrivetrainSubsystem m_drivetrainSubsystem; 
  double gravityRatio; // current gyro z / gyro z at robot init
  double isTippedThreshold = 0.95; // for gravityRatio
  double untippedDriveSpeed = 0.15;
  double tippedDriveSpeed = 0.1;
  double waitTime = 1; // seconds
  boolean isTipped; // for triggering isFinished() at the right time

  public SwerveDriveBalanceJerky(DrivetrainSubsystem ds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = ds;
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gravityRatio = m_drivetrainSubsystem.getGyroscope().getRawAccelZ() / Robot.initialGravityZAccel;

    if (gravityRatio > isTippedThreshold) { // if not yet tipped
      m_drivetrainSubsystem.drive(untippedDriveSpeed, 0, 0, false);

    } else {
      isTipped = true;
      m_drivetrainSubsystem.drive(tippedDriveSpeed, 0, 0, false);
      WaitCommand wc = new WaitCommand(waitTime);
      wc.execute(); 
    }
    /* 0. Check when you've reached station (check the FEEDBACK command version) one for gravity ratio
     * 1. Drive forward a teeny bit - ds.drive()
     * 2. check pitch angle
     * 3. as soon as pitch angle sign flips go back a teeny bit (in isFinished())
     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isTipped = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isTipped && gravityRatio > isTippedThreshold;
  }
}
