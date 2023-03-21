// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Pose;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Poser;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import frc.robot.subsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

/** Add your docs here. */
public class AutonomousCommands  {
    public HashMap<String, Command> eventMap = new HashMap<>();
    public final double AUTO_MAX_VEL = 4.0; //4.0; 
    public final double AUTO_MAX_ACCEL = 3.0; //3.0;

    DrivetrainSubsystem m_dts;
    ArmPneumaticSubsystem m_aps;
    ArmMotorSubsystem m_ams;
    ClawPneumaticSubsystem m_cps;
    ObjectTrackerSubsystem m_otsg;
    ObjectTrackerSubsystem m_otsc;

    Command m_highScoreCommand; 

    Poser m_getPose;

    public Command scoreHigh() {
        Command c = new SequentialCommandGroup(
            new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE + 20)),
            new MoveArmToPoseCommand(m_aps, m_ams, RobotContainer.m_getPose),

            // // TODO: @Darren / @Megan, with the new transitions and the shorter arm,
            // // we may need to wait for a bit of time here for the pneumatics to stop moving
            // // in MoveArmToPoseCommand, or else while the pneumatics are still moving, it could
            // // open up the claw... this really only applies to chained autonomous commands
            // // where you move the arm and then score immediately without the human in the loop.
            // // If the human in the loop commands the actual gripper release, then won't need 
            // // a wait after MoveArmToPoseCommand()
            new WaitCommand(0.5),
            // new ArmMovementCommand(m_ams, Constants.TOP_SCORING_ANGLE + 20),
            new ParallelCommandGroup(
                new ClawPneumaticCommand(m_cps, true),
                new PrintCommand("**********end of scoreHigh()")
            )
        );
        return c;
    }

    public AutonomousCommands(DrivetrainSubsystem dts, ArmPneumaticSubsystem aps, ArmMotorSubsystem ams, ClawPneumaticSubsystem cps, ObjectTrackerSubsystem otsg, ObjectTrackerSubsystem otsc, Poser p) {
        m_dts = dts;
        m_aps = aps;
        m_ams = ams; 
        m_cps = cps;
        m_otsg = otsg;
        m_otsc = otsc;
        m_getPose = p;

        m_highScoreCommand = scoreHigh();

        eventMap.put("score high", scoreHigh());
        eventMap.put("event2", new PrintCommand("Passed marker 2"));
    }

    public Command DoNothing() {
        return null;
    }

    public Command SwerveAutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrainSubsystem.followPath()),
            new SwerveAutoBalanceCommandFEEDBACK(drivetrainSubsystem),
            new PrintCommand("IN SABC"),
            new InstantCommand(() -> drivetrainSubsystem.followJoystick())
        );
    }

    public void PrintNTDataString(){
        RobotContainer.m_objectTrackerSubsystemChassis.data();
        RobotContainer.m_objectTrackerSubsystemGripper.data();
    }

    public Command OutPath(DrivetrainSubsystem drivetrainSubsystem) {
        m_dts.zeroOdometry();
        PathPlannerTrajectory traj = PathPlanner.loadPath("Out Path", new PathConstraints(0.5, 0.1));
        Command c = drivetrainSubsystem.followTrajectoryCommand(traj, true);
        return c;
    }

    public Command RotationTesting(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Rotation Testing", new PathConstraints(0.02, 0.05));
        return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    }

    // public Command LeftScoreTwiceEngage(DrivetrainSubsystem drivetrainSubsystem) {
    //     PathPlannerTrajectory traj = PathPlanner.loadPath("Left score twice engage", new PathConstraints(0.02, 0.05));
    //     return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    // }

    // public Command MidScoreTwiceEngage(DrivetrainSubsystem drivetrainSubsystem) {
    //     PathPlannerTrajectory traj = PathPlanner.loadPath("Mid score twice engage", new PathConstraints(0.02, 0.05));
    //     return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    // }

    // public Command RightScoreTwiceEngage(DrivetrainSubsystem drivetrainSubsystem) {
    //     PathPlannerTrajectory traj = PathPlanner.loadPath("Right score twice engage", new PathConstraints(0.02, 0.05));
    //     return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    // }

    // public Command RightScoreEngage(DrivetrainSubsystem drivetrainSubsystem) {
    //     PathPlannerTrajectory traj = PathPlanner.loadPath("Right score engage", new PathConstraints(0.02, 0.05));
    //     return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    // }

    // public Command MidScoreEngage(DrivetrainSubsystem drivetrainSubsystem) {
    //     PathPlannerTrajectory traj = PathPlanner.loadPath("Mid score engage", new PathConstraints(0.02, 0.05));
    //     return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    // }

    // public Command LeftScoreEngage(DrivetrainSubsystem drivetrainSubsystem) {
    //     PathPlannerTrajectory traj = PathPlanner.loadPath("Left score engage", new PathConstraints(2, 1));
    //     return drivetrainSubsystem.followTrajectoryCommand(traj, true);
    // }

    public Command CircleTest(DrivetrainSubsystem drivetrainSubsystem) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Circle test", new PathConstraints(2, 1));
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

    public Command scoreHighOutEngageLeftCone() {
        PathPlannerTrajectory path = PathPlanner.loadPath("Score left out engage cone", new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL));
        
        Command backHome = new SequentialCommandGroup(
            new SetTargetPoseCommand(new Pose(Constants.HOME_EXTEND, Constants.HOME_ARM_ANGLE)),
            new MoveArmToPoseCommand(m_aps, m_ams, RobotContainer.m_getPose)
        );

        Command c = new SequentialCommandGroup(
            m_highScoreCommand,
            new ParallelCommandGroup(
                m_dts.followTrajectoryCommand(path, true),
                backHome
            ),
            new SwerveAutoBalanceCommand(m_dts)
        );

        return c;   
    }

    public Command scoreHighDriveOut() { // with on the fly generated paths
        m_dts.zeroOdometry();
        PathPlannerTrajectory traj = PathPlanner.generatePath(
            new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(4.5, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0))
        );

        Command c = m_dts.followTrajectoryCommand(traj, true);
        Command backHome = new SequentialCommandGroup(
            new SetTargetPoseCommand(new Pose(Constants.HOME_EXTEND, Constants.HOME_ARM_ANGLE)),
            new MoveArmToPoseCommand(m_aps, m_ams, RobotContainer.m_getPose)
        );

        Command s = new SequentialCommandGroup(scoreHigh(), new ParallelCommandGroup(c, backHome));
        return s;
    }

    public Command scoreHighMobilityGrabScoreHigh() {
        m_dts.zeroOdometry();
        PathPlannerTrajectory traj = PathPlanner.generatePath(
            new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(1.5, 0.2), Rotation2d.fromRadians(0), Rotation2d.fromRadians((Math.PI))),
            new PathPoint(new Translation2d(3.7, 0.4), Rotation2d.fromRadians(0), Rotation2d.fromRadians((Math.PI)+0.95))
        );

        PathPlannerTrajectory traj2 = PathPlanner.generatePath(
            new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(1.5, -0.05), Rotation2d.fromRadians(0), Rotation2d.fromRadians((Math.PI))),
            new PathPoint(new Translation2d(3.8, -0.10), Rotation2d.fromRadians(0), Rotation2d.fromRadians((Math.PI)+0.95)),
            new PathPoint(new Translation2d(4.2, -0.10), Rotation2d.fromRadians(0), Rotation2d.fromRadians((Math.PI)+0.95))

            // new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            // new PathPoint(new Translation2d(1.5, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(-1.0*Math.PI-0.001)),
            // new PathPoint(new Translation2d(3.5, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(-1.0*Math.PI-0.001 - 0.9))

            // new PathPoint(new Translation2d(3.5, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians((Math.PI)+1)), // position, heading(direction of travel)
            // new PathPoint(new Translation2d(1.5, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians((Math.PI))),
            // new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0))
        );

        Command c = new SequentialCommandGroup( new WaitCommand(0.5),
                                                m_dts.followTrajectoryCommand(traj, true));
        Command c2 = m_dts.followTrajectoryCommand(traj2, false);

        Command pickUpMid = new SequentialCommandGroup(
            new ArmPneumaticCommand(m_aps, false),
            new WaitCommand(1),
            // new SetTargetPoseCommand(new Pose(Constants.ARM_EXTEND_PICKUP_FLOOR, Constants.ARM_ANGLE_PICKUP_FLOOR)),
            new SetTargetPoseCommand(new Pose(Constants.ARM_EXTEND_PICKUP_FLOOR, Constants.BOTTOM_SCORING_ANGLE)),
            new MoveArmToPoseCommand(m_aps, m_ams, RobotContainer.m_getPose)
        );

        Command s = new SequentialCommandGroup( scoreHigh(), 
                                                new ParallelCommandGroup(   c,
                                                    
                                                pickUpMid), 
                                                new SequentialCommandGroup( new InstantCommand(() -> m_dts.followPath()),
                                                                            new VisionDriveClosedLoopCommand(Constants.TARGET_OBJECT_LABEL_CUBE, true, m_dts, m_otsc, true),
                                                                            new PrintCommand("Reached the cube"),
                                                                            new AlignGripperToObjectCommand(m_dts, m_otsg, m_aps, m_cps),
                                                                            new InstantCommand(() -> m_dts.followJoystick())),
                                                // pneumatic and then wait so the claw can close
                                                // we may be able to take that wait down abit
                                                new ClawPneumaticCommand(m_cps, false),
                                                new WaitCommand(0.5),
                                                // put the arm in a scoring position
                                                // should change this to upper
                                                new SetTargetPoseCommand(new Pose(Constants.TOP_SCORING_EXTEND, Constants.TOP_SCORING_ANGLE)),
                                                new MoveArmToPoseCommand(
                                                    m_aps, 
                                                    m_ams, 
                                                    m_getPose
                                                ),
                                                c2,
                                                // if we have it, april tag code goes here.

                                                // release and score
                                                new ClawPneumaticCommand(m_cps, true)
                                            
                                            );
        return s;
    }

    /**
     * PATHS WE NEED
     * score high out engage left - cone and cube
     * score high out engage right - cone and cube
     * score high engage, no leaving the field, cone and cube in co-op grid
     */
    
     public Command scoreHighOutScoreSustationSIDE() { 
        PathPlannerTrajectory path = PathPlanner.loadPath("Score right out engage", new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL));
        
        Command backHome = new SequentialCommandGroup(
            new SetTargetPoseCommand(new Pose(Constants.HOME_EXTEND, Constants.HOME_ARM_ANGLE)),
            new MoveArmToPoseCommand(m_aps, m_ams, RobotContainer.m_getPose)
        );

        Command c = new SequentialCommandGroup(
            scoreHigh(),
            new ParallelCommandGroup(
                m_dts.followTrajectoryCommand(path, true),
                backHome
            ),
            new SwerveAutoBalanceCommand(m_dts)
        );

        return c;   

     }



}
