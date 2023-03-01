// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PPLogging;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveStraightCommand extends CommandBase {
  private boolean m_allDone = false;
  DrivetrainSubsystem m_driveTrainSubsystem;
  Command m_c = null;

  /** Creates a new DriveStraightCommand. */
  public DriveStraightCommand(DrivetrainSubsystem dts) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrainSubsystem = dts;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Simple path without holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
    PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(0.1, 0.1), 
        new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
        new PathPoint(new Translation2d(1.0, 0.0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0))//6 * Math.PI / 3.09)) // position, heading(direction of travel)
        // new PathPoint(new Translation2d(0, 1), Rotation2d.fromRadians(0) // position, heading(direction of travel)
    );
    m_c = m_driveTrainSubsystem.followTrajectoryCommand(traj, true);

    m_c.initialize();
  }

  // private static Translation2d mapCoordinates(double x, double y) {
  //   return new Translation2d(-(y / 3), x / 3);
  // }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_c != null) {
      m_c.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_c != null) {
      m_c.end(interrupted);
    }  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_allDone || m_c == null || m_c.isFinished();
  }
}
