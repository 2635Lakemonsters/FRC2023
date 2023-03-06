// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Pose;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import frc.robot.subsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class AutonomousCommands  {
    public HashMap<String, Command> eventMap = new HashMap<>();
    public final double AUTO_MAX_VEL = 4; 
    public final double AUTO_MAX_ACCEL = 3;

    DrivetrainSubsystem m_dts;
    ArmPneumaticSubsystem m_aps;
    ArmMotorSubsystem m_ams;
    ClawPneumaticSubsystem m_cps;

    private Command scoreHigh() {
        Command c = new SequentialCommandGroup(
            new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE)),
            new MoveArmToPoseCommand(m_aps, m_ams, RobotContainer.m_getPose),

            new ParallelCommandGroup(
                new ClawPneumaticCommand(m_cps, false),
                new PrintCommand("**********end of scoreHigh()")
            )
        );
        return c;
    }

    public AutonomousCommands(DrivetrainSubsystem dts, ArmPneumaticSubsystem aps, ArmMotorSubsystem ams, ClawPneumaticSubsystem cps) {
        m_dts = dts;
        m_aps = aps;
        m_ams = ams; 
        m_cps = cps;

        eventMap.put("score high", scoreHigh());
        eventMap.put("event2", new PrintCommand("Passed marker 2"));
    }

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
        return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    }

    public Command BottomScoreTwice(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Bottom score twice engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    }

    public Command MidScoreTwice(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Mid score twice engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    }

    public Command TopScoreTwice(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Top score twice engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    }

    public Command ScoreTopGrab(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Score top grab engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    }

    public Command ScoreMidGrab(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Score mid grab engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    }

    public Command ScoreBottomGrab(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Score bottom grab engage", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj, true);
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

    public Command autoPathMarkerCommand() {
        PathPlannerTrajectory path = PathPlanner.loadPath("Score left out engage", new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL));
        // FollowPathWithEvents command = new FollowPathWithEvents(
        //     m_dts.followTrajectoryCommand(path, true),
        //     path.getMarkers(),
        //     eventMap
        // );
        // return command; 
        Command high = scoreHigh();
        Command c = new SequentialCommandGroup(
            high,
            m_dts.followTrajectoryCommand(path, true)
        );

        return c; 
    }

}
