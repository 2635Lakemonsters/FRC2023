// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Command to balance/engage ONLY in auto. Disables joysticks to avoid the crazy dangerous driving 
 * we saw at wilsonville
 * 
 * Should be active only when the button is held, canceled when released. In teleop this command 
 * will NOT do anything UNLESS the robot is already titled, in which case it will drive forward(?)
 */
public class SwerveAutoBalanceCommandFEEDBACK extends CommandBase {

  private DrivetrainSubsystem m_drivetrainSubsystem;
  double ff_gain = 0.01;
  public double x_feedforward_final; 

  // for the drive 54 inches forward methodd
  double current_pitch;
  double start_loc; // only in x direction, forward/back
  double dist_travelled;
  PIDController controller = new PIDController(0.01, 0, 0);
  double forward_output;

  boolean reached_station; // changes to TRUE when the pitch angle changes

  /** 
   * SwerveAutoBalanceCommandFEEDBACK. Starts the pid loop when the pitch angle of bot changes
   */
  public SwerveAutoBalanceCommandFEEDBACK(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.followPath(); // disable joysticks just in case
    start_loc = m_drivetrainSubsystem.getPose().getTranslation().getX();
    reached_station = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double gravity_ratio = m_drivetrainSubsystem.m_gyro.getRawAccelZ() / Robot.init_gyro_z_accel; // between current read vals and vals when bot intialized
    // double sin = Math.sqrt(1 - gravity_ratio * gravity_ratio) * ff_gain;
    // x_feedforward_final = Math.copySign(sin, m_drivetrainSubsystem.getGyroscope().getRoll());

    // m_drivetrainSubsystem.drive(x_feedforward_final, 0, 0, false);
    //System.out.println(1.7 * NavX.getXAccelFiltered() - 0.008 * NavX.getRawGyroY());

    /*FF
     * get the z accelerometer component of the gyro and ratio with the force of gravity in robot init -> gives us the cosine of the angle
     * 
     * FB
     * 
     * 54.5 in to get to center
     */
    // current_pitch = m_drivetrainSubsystem.getPitch();

    // drive slowly forward until pitch angle changes, then move on to execute() where position pid loop takes over
    if (!reached_station) {
      m_drivetrainSubsystem.drive(0.05, 0, 0, false);
      double gravity_ratio = m_drivetrainSubsystem.m_gyro.getRawAccelZ() / Robot.init_gyro_z_accel;
    
      if (Math.abs(gravity_ratio) < 0.95) {
        reached_station = true;
      }
    } else {
      dist_travelled = m_drivetrainSubsystem.getPose().getTranslation().getX() - start_loc;
      controller.setSetpoint(54.5);
      forward_output = controller.calculate(dist_travelled);
      m_drivetrainSubsystem.drive(forward_output, 0, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.followJoystick();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
