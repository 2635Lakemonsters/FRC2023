// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class AutonomousCommands  {
    public Command DoNothing() {
        return null;
    }

    public void PrintNTDataString(){
        RobotContainer.m_objectTrackerSubsystemChassis.data();
        RobotContainer.m_objectTrackerSubsystemGripper.data();
    }

    public Command OutPath(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Out Path", new PathConstraints(0.1, 0.1));
        Command c = drivetrainSubsystem.followTrajectoryCommand(traj, true);
        return c;
    }

    public Command RotatePath(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Rotation Testing", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj);
    }

    public Command BottomScoreTwice(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Bottom score twice engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj);
    }

    public Command MidScoreTwice(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Mid score twice engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj);
    }

    public Command TopScoreTwice(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Top score twice engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj);
    }

    public Command ScoreTopGrab(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Score top grab engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj);
    }

    public Command ScoreMidGrab(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Score mid grab engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj);
    }

    public Command ScoreBottomGrab(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Score bottom grab engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj);
    }

    /** Drive straight with AutonomousTrajectoryCommand and path planner traj */
    public Command driveStraightPP(DrivetrainSubsystem ds) {
        PathPlannerTrajectory traj = PathPlanner.generatePath(
            new PathConstraints(0.5, 0.5), 
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(0, 1), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0))//6 * Math.PI / 3.09)) // position, heading(direction of travel)
            // new PathPoint(new Translation2d(0, 1), Rotation2d.fromRadians(0) // position, heading(direction of travel)
        );

        AutonomousTrajectoryCommand atc = new AutonomousTrajectoryCommand(ds, traj);
        return atc.runAutonomousCommand();
    }

    /** Drive straight with AutonomousTrajectoryCommand and normal traj */
    public Command driveStraight(DrivetrainSubsystem ds) {
        AutonomousTrajectoryCommand atc = new AutonomousTrajectoryCommand(ds);
        return atc.runAutonomousCommand();
    }
}
