// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** SwerveDriveBalanceJerky
 * </p> Jerk move step by step until balance tips. Then move one step back.  */
public class SwerveDriveBalanceJerky extends CommandBase {
  DrivetrainSubsystem m_drivetrainSubsystem; 
  double gravityRatio; // current gyro z / gyro z at robot init
  double roll;
  double isTippedThreshold = 5; // for gravityRatio
  double untippedDriveSpeed = 0.5;
  double tippedDriveSpeed = 0.1;
  double waitTime = 1; // seconds
  boolean isTipped; // for triggering isFinished() at the right time

  PIDController controller = new PIDController(0.01, 0, 0);
  double center = 54.5;
  double startXPos;
  double ff_gain = 0.01; // TODO tune

  double inchForwardBy = 2;

  LinearFilter filter = LinearFilter.movingAverage(50);

  public SwerveDriveBalanceJerky(DrivetrainSubsystem ds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = ds;
    isTipped = false;
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startXPos = m_drivetrainSubsystem.getPose().getTranslation().getX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gravityRatio = filter.calculate(m_drivetrainSubsystem.getGyroscope().getRawAccelZ() / Robot.initialGravityZAccel);
    SmartDashboard.putNumber("Gravity Ratio", gravityRatio);
    roll = filter.calculate(m_drivetrainSubsystem.getGyroscope().getRoll() / Robot.initGyroRoll); //roll is pitch
    SmartDashboard.putNumber("Gyro Ratio", roll);
    if (roll < isTippedThreshold) { // if not yet nose up
      m_drivetrainSubsystem.drive(untippedDriveSpeed, 0, 0, false);

    } else {
      m_drivetrainSubsystem.drive(tippedDriveSpeed, 0, 0, false);
      isTipped = true;
      double currentXPos = m_drivetrainSubsystem.getPose().getTranslation().getX();

      if (isCurrentlyTipped()) {
        controller.setSetpoint(currentXPos + inchForwardBy);
      }
      
      // calculate feedforward (sine)
      double sin = Math.sqrt(1 - roll * roll);
      double feedForward = Math.copySign(sin, m_drivetrainSubsystem.getGyroscope().getRoll()) * ff_gain;

      // feedback + drive
      double forward_output_fb = controller.calculate(currentXPos - startXPos);
      // m_drivetrainSubsystem.drive(forward_output_fb + feedForward, 0, 0, false);
    }

    /* 0. Check when you've reached station (check the FEEDBACK command version) one for gravity ratio
     * 1. Drive forward a teeny bit - ds.drive()
     * 2. check pitch angle
     * 3. as soon as pitch angle sign flips go back a teeny bit (in isFinished())
     */
  }

  private boolean isCurrentlyTipped() {
    return roll > isTippedThreshold;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isTipped = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return isTipped && isCurrentlyTipped();
    return false;
  }
}
