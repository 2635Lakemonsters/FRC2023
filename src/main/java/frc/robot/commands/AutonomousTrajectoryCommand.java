// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.AutoConstants;
// import frc.robot.subsystems.DrivetrainSubsystem;

// /** Add your docs here. */
// public class AutonomousTrajectoryCommand {
//     DrivetrainSubsystem m_drivetrainSubsystem;
//     Trajectory m_traj; 

//     public AutonomousTrajectoryCommand(DrivetrainSubsystem dts, Trajectory traj) {
//         m_drivetrainSubsystem = dts;
//         m_traj = traj; 
//     }

//     public AutonomousTrajectoryCommand(DrivetrainSubsystem dts) {
//         m_drivetrainSubsystem = dts;
//     }

//     /**
//    * Use this method to pass the autonomous command to the main {@link Robot} class.
//    * 
//    * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/RobotContainer.java
//    * 
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     // Create config for trajectory
//     TrajectoryConfig config =
//         new TrajectoryConfig(
//                 AutoConstants.kMaxSpeedMetersPerSecond,
//                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//             // Add kinematics to ensure max speed is actually obeyed
//             .setKinematics(m_drivetrainSubsystem.getSwerveDriveKinematics());

//     if (m_traj == null) {
//         this.generateDefaultTrajectory(config); 
//     }

//     var thetaController =
//         new ProfiledPIDController(
//             AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     SwerveControllerCommand swerveControllerCommand =
//         new SwerveControllerCommand(
//             m_traj,
//             this::getPose, // Functional interface to feed supplier
//             m_drivetrainSubsystem.getSwerveDriveKinematics(),

//             // Position controllers
//             new PIDController(AutoConstants.kPXController, 0, 0),
//             new PIDController(AutoConstants.kPYController, 0, 0),
//             thetaController,
//             this::setModuleStates,
//             m_drivetrainSubsystem);

//     // Reset odometry to the starting pose of the trajectory.
//     m_drivetrainSubsystem.resetOdometry(m_traj.getInitialPose());

//     // Run path following command, then stop at the end.
//     return swerveControllerCommand.andThen(() -> m_drivetrainSubsystem.drive(0, 0, 0, false));
//   }
  
//   public Trajectory generateDefaultTrajectory(TrajectoryConfig trajConfig) {
//     // An example trajectory to follow.  All units in meters.
//     Trajectory exampleTrajectory =
//         TrajectoryGenerator.generateTrajectory(
//             // Start at the origin facing the +X direction
//             new Pose2d(0, 0, new Rotation2d(0)),
//             // Pass through these two interior waypoints, making an 's' curve path
//             List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//             // End 3 meters straight ahead of where we started, facing forward
//             new Pose2d(3, 0, new Rotation2d(0)),
//             trajConfig);
    
//     return exampleTrajectory; 
//   }

// }
