// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Class to follow an autonomous trajectory with non-pathplanner SwerveControllerCommand
 * </p> *Probably misnamed tbh*
 * </p> Call runAutonomousCommand() to actually run the path you pass in when you 
 * create an AutonomousTrajectoryCommand object
 * </p> Should be able to take in and follow a PathPlanner trajectory.
 */
public class AutonomousTrajectoryCommand {
    DrivetrainSubsystem m_drivetrainSubsystem;
    Trajectory m_traj; 

    double kMaxSpeedMetersPerSecond = 0.5;
    double kMaxAccelerationMetersPerSecondSquared = 0.5;
    double kMaxAngularSpeedRadiansPerSecond = Math.PI / 10;
    double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 10;

    double kPXController = 1; // forward 
    double kPYController = 1; // strafe 
    double kPThetaController = 1; // rotation

    // Constraint for the motion profiled robot angle controller
    TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    
    // trajectory configuration w/ max speed and accel
    TrajectoryConfig config;

    /** Generates new AutonomousTrajectoryCommand
     * @param dts DrivetrainSubsystem requirement
     * @param traj Autonomous Trajectory 
     */
    public AutonomousTrajectoryCommand(DrivetrainSubsystem dts, Trajectory traj) {
        m_drivetrainSubsystem = dts;
        m_traj = traj; 
    }

    /** Default constructor, uses a preset trajectory in the AutonomousTrajectoryCommand class */
    public AutonomousTrajectoryCommand(DrivetrainSubsystem dts) {
        m_drivetrainSubsystem = dts;
    }

    /** Use this method to pass the autonomous command to the main {@link Robot} class.
     * </p> https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/RobotContainer.java
     * @return the SwerveControllerCommand to run in autonomous
    */
    public Command runAutonomousCommand() {
        config =
        new TrajectoryConfig(
            this.kMaxSpeedMetersPerSecond,
            this.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_drivetrainSubsystem.getSwerveDriveKinematics());
        
        // Create config for trajectory. Sets default trajectory if using the non-traj taking constructory
        if (m_traj == null) {
            m_traj = this.generateDefaultTrajectory(); 
        }

        var thetaController =
            new ProfiledPIDController(
                this.kPThetaController, 0, 0, this.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                m_traj,
                m_drivetrainSubsystem::getPose, // Functional interface to feed supplier
                m_drivetrainSubsystem.getSwerveDriveKinematics(),

                // Position controllers
                new PIDController(this.kPXController, 0, 0),
                new PIDController(this.kPYController, 0, 0),
                thetaController,
                m_drivetrainSubsystem::setModuleStates,
                m_drivetrainSubsystem);

        // Reset odometry to the starting pose of the trajectory.
        m_drivetrainSubsystem.resetOdometry(m_traj.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_drivetrainSubsystem.drive(0, 0, 0, false));
    } 
  
    private Trajectory generateDefaultTrajectory() {
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1.0 / 3.0, 0, new Rotation2d(0)),
            config);
    
    return exampleTrajectory; 
  }

  public Trajectory generateZeroTrajectory() {
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(0)),
            config);

    return exampleTrajectory; 
  }

  public String getTrajectoryWaypoints() {
    return m_traj.toString();
  }

}
