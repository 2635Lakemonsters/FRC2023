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
    public final double AUTO_MAX_VEL = 10. * 12. / Constants.INCHES_PER_METER * 0.6; //4.0; 
    public final double AUTO_MAX_ACCEL = 3.0 * 0.6; //3.0;

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

    public void PrintNTDataString(){
        RobotContainer.m_objectTrackerSubsystemChassis.data();
        RobotContainer.m_objectTrackerSubsystemGripper.data();
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

        Command s = new SequentialCommandGroup( scoreHigh(), 
                                                new ParallelCommandGroup(c, backHome));
        return s;
    }
    
    private double totalRotation = Math.PI + 0.65;
    private double outDistance = 4.0;
    private double returnDistance = 3.5;
    

    public Command trajectoryTest() {
        totalRotation = Math.PI + 0.65;

        m_dts.zeroOdometry();
        PathPlannerTrajectory traj = PathPlanner.generatePath(
            // new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            // new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(Math.PI/4), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            // new PathPoint(new Translation2d(outDistance/2., 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation/2.)),
            // new PathPoint(new Translation2d(outDistance, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation))
            new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(0.), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(outDistance/2., 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation/2.)),
            new PathPoint(new Translation2d(outDistance, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation))
        );
       

        PathPlannerTrajectory traj2 = PathPlanner.generatePath(
            // new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            // new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(Math.PI/6), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            // new PathPoint(new Translation2d(returnDistance/2., -0.15), Rotation2d.fromRadians(Math.PI/6), Rotation2d.fromRadians(totalRotation/2.)),
            // new PathPoint(new Translation2d(returnDistance, -0.35 - 0.05), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation))
            new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            new PathPoint(new Translation2d(outDistance, 0), Rotation2d.fromRadians(0.), Rotation2d.fromRadians(totalRotation)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(outDistance-outDistance/2., 0.), Rotation2d.fromRadians(0.), Rotation2d.fromRadians(totalRotation/2.)),
            new PathPoint(new Translation2d(outDistance-outDistance, 0.), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0.))
        );

        Command c = m_dts.followTrajectoryCommand(traj, true);
        Command c2 = m_dts.followTrajectoryCommand(traj2, false);

        Command s = new SequentialCommandGroup(
            c,
            // new WaitCommand(2.0),
            // c2,
            new WaitCommand(2.0)
         );
        return s;

    }

    public Command trajectoryTestNegative() {
        totalRotation = -Math.PI - 0.65;

        m_dts.zeroOdometry();
        PathPlannerTrajectory traj = PathPlanner.generatePath(
            // new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            // new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(Math.PI/4), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            // new PathPoint(new Translation2d(outDistance/2., 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation/2.)),
            // new PathPoint(new Translation2d(outDistance, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation))
            new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(0.), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(outDistance/2., 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation/2.)),
            new PathPoint(new Translation2d(outDistance, 0), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation))
        );
       

        PathPlannerTrajectory traj2 = PathPlanner.generatePath(
            // new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            // new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(Math.PI/6), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            // new PathPoint(new Translation2d(returnDistance/2., -0.15), Rotation2d.fromRadians(Math.PI/6), Rotation2d.fromRadians(totalRotation/2.)),
            // new PathPoint(new Translation2d(returnDistance, -0.35 - 0.05), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation))
            new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            new PathPoint(new Translation2d(outDistance, 0), Rotation2d.fromRadians(0.), Rotation2d.fromRadians(totalRotation)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(outDistance-outDistance/2., 0.), Rotation2d.fromRadians(0.), Rotation2d.fromRadians(totalRotation/2.)),
            new PathPoint(new Translation2d(outDistance-outDistance, 0.), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0.))
        );

        Command c = m_dts.followTrajectoryCommand(traj, true);
        Command c2 = m_dts.followTrajectoryCommand(traj2, false);

        Command s = new SequentialCommandGroup(
            c,
            // new WaitCommand(2.0),
            c2,
            new WaitCommand(2.0)
         );
        return s;

    }

    public Command scoreHighMobilityGrabScoreHighRight() {
        // rotate in towards field by negative rotation
        totalRotation = Math.PI + 0.75;

        m_dts.zeroOdometry();
        PathPlannerTrajectory traj = PathPlanner.generatePath(
            new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(Math.PI / 6.), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(outDistance/2., 0.35), Rotation2d.fromRadians(Math.PI / 12. * 0.), Rotation2d.fromRadians(totalRotation/2.)),
            
            new PathPoint(new Translation2d(outDistance, 0.55), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation))
        );
       

        PathPlannerTrajectory traj2 = PathPlanner.generatePath(
            new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(Math.PI / 6.), Rotation2d.fromRadians(totalRotation)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(returnDistance/2., -0.2), Rotation2d.fromRadians(0), Rotation2d.fromRadians(totalRotation/2.+Math.PI/16)),
            new PathPoint(new Translation2d(returnDistance, -0.4), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0. + Math.PI/8))
        );

        Command c = new SequentialCommandGroup( new WaitCommand(0.5),
                                                m_dts.followTrajectoryCommand(traj, true));
        Command c2 = m_dts.followTrajectoryCommand(traj2, false);

        Command pickUpMid = new SequentialCommandGroup(
            new ArmPneumaticCommand(m_aps, false),
            new ArmMovementCommand(m_ams, 190),
            new WaitCommand(1.5), // this makes the arm not hit the ref
            // new SetTargetPoseCommand(new Pose(Constants.ARM_EXTEND_PICKUP_FLOOR, Constants.ARM_ANGLE_PICKUP_FLOOR)),
            new SetTargetPoseCommand(new Pose(Constants.ARM_EXTEND_PICKUP_FLOOR, Constants.ARM_ANGLE_PICKUP_FLOOR)),
            new MoveArmToPoseCommand(m_aps, m_ams, RobotContainer.m_getPose)
        );

        Command s = new SequentialCommandGroup( new ClawPneumaticCommand(m_cps, false), scoreHigh(), 
                                                new ParallelCommandGroup(c, pickUpMid), 
                                                new SequentialCommandGroup( new InstantCommand(() -> m_dts.followPath()),
                                                                            new VisionDriveClosedLoopCommand(Constants.TARGET_OBJECT_LABEL_CUBE, true, m_dts, m_otsc, false),
                                                                            new PrintCommand("Reached the cube"),
                                                                            new AlignGripperToObjectCommand(m_dts, m_otsg, m_aps, m_cps),
                                                                            new InstantCommand(() -> m_dts.followJoystick())),
                                                // pneumatic and then wait so the claw can close
                                                // we may be able to take that wait down abit
                                                new ClawPneumaticCommand(m_cps, false),
                                                new ArmPneumaticCommand(m_aps, false),
                                                new WaitCommand(1.5),
                                                // put the arm in a scoring position
                                                // should change this to upper
                                                new SetTargetPoseCommand(new Pose(false, Constants.TOP_SCORING_ANGLE)),
                                                new ParallelCommandGroup(
                                                    new MoveArmToPoseCommand(
                                                    m_aps, 
                                                    m_ams, 
                                                    m_getPose
                                                    ),
                                                    c2
                                                ),
                                                // if we have it, april tag code goes here.
                                                new ParallelCommandGroup(
                                                    new SequentialCommandGroup( new InstantCommand(()->m_dts.followPath()),
                                                                                new VisionDriveClosedLoopCommand(Constants.TARGET_OBJECT_LABEL_APRIL_TAG, true, m_dts, m_otsc, false, 3),
                                                                                // new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.offsetFromAprilTagToCenter, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG, true),
                                                                                // new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.offsetFromAprilTagToCenter, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG, false),
                                                                                new InstantCommand(()->m_dts.followJoystick())
                                                                            ),
                                                    // new MoveToScore(m_dts, m_otsc, 0, 0, false),
                                                    new ArmPneumaticCommand(m_aps, true)
                                                )
                                                
                                                // , 
                                                // // No Time to tune lining up for release and score
                                                // // release and score
                                                // new ClawPneumaticCommand(m_cps, true)
                                            
                                            );
        return s;
    }

    public Command scoreHighMobilityGrabScoreHighLeft() {
        // rotate in towards field by negative rotation
        totalRotation = Math.PI + 0.75;

        m_dts.zeroOdometry();
        PathPlannerTrajectory traj = PathPlanner.generatePath(
            new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(Math.PI / 6.), Rotation2d.fromRadians(0)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(outDistance/2., -0.50), Rotation2d.fromRadians(Math.PI / 12.), Rotation2d.fromRadians(-totalRotation/2.)),
            
            new PathPoint(new Translation2d(outDistance, -0.65), Rotation2d.fromRadians(0), Rotation2d.fromRadians(-totalRotation))
        );
       

        PathPlannerTrajectory traj2 = PathPlanner.generatePath(
            new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL), 
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromRadians(Math.PI / 6.), Rotation2d.fromRadians(-totalRotation)), // position, heading(direction of travel)
            new PathPoint(new Translation2d(returnDistance/2., 0.), Rotation2d.fromRadians(0), Rotation2d.fromRadians(-totalRotation/2.)),
            new PathPoint(new Translation2d(returnDistance, 0.), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0.))
        );

        Command c = new SequentialCommandGroup( new WaitCommand(0.5),
                                                m_dts.followTrajectoryCommand(traj, true));
        Command c2 = m_dts.followTrajectoryCommand(traj2, false);

        Command pickUpMid = new SequentialCommandGroup(
            new ArmPneumaticCommand(m_aps, false),
            new ArmMovementCommand(m_ams, 190),
            new WaitCommand(1), // this makes the arm not hit the ref
            // new SetTargetPoseCommand(new Pose(Constants.ARM_EXTEND_PICKUP_FLOOR, Constants.ARM_ANGLE_PICKUP_FLOOR)),
            new SetTargetPoseCommand(new Pose(Constants.ARM_EXTEND_PICKUP_FLOOR, Constants.ARM_ANGLE_PICKUP_FLOOR)),
            new MoveArmToPoseCommand(m_aps, m_ams, RobotContainer.m_getPose)
        );

        Command s = new SequentialCommandGroup( scoreHigh(), 
                                                new ParallelCommandGroup(c, pickUpMid), 
                                                new SequentialCommandGroup( new InstantCommand(() -> m_dts.followPath()),
                                                                            new VisionDriveClosedLoopCommand(Constants.TARGET_OBJECT_LABEL_CUBE, true, m_dts, m_otsc, true),
                                                                            new PrintCommand("Reached the cube"),
                                                                            new AlignGripperToObjectCommand(m_dts, m_otsg, m_aps, m_cps),
                                                                            new InstantCommand(() -> m_dts.followJoystick())),
                                                // pneumatic and then wait so the claw can close
                                                // we may be able to take that wait down abit
                                                new ClawPneumaticCommand(m_cps, false),
                                                new ArmPneumaticCommand(m_aps, false),
                                                new WaitCommand(0.5),
                                                // put the arm in a scoring position
                                                // should change this to upper
                                                new SetTargetPoseCommand(new Pose(false, Constants.TOP_SCORING_ANGLE)),
                                                new ParallelCommandGroup(
                                                    new MoveArmToPoseCommand(
                                                    m_aps, 
                                                    m_ams, 
                                                    m_getPose
                                                    ),
                                                    c2
                                                ),
                                                // if we have it, april tag code goes here.
                                                new ParallelCommandGroup(
                                                    new SequentialCommandGroup( new InstantCommand(()->m_dts.followPath()),
                                                                                new VisionDriveClosedLoopCommand(Constants.TARGET_OBJECT_LABEL_APRIL_TAG, true, m_dts, m_otsc, false, 1),
                                                                                // new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.offsetFromAprilTagToCenter, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG, true),
                                                                                // new MoveToScore(m_drivetrainSubsystem, m_objectTrackerSubsystemChassis, Constants.offsetFromAprilTagToCenter, Constants.FIELD_OFFSET_FROM_NODE_TO_APRILTAG, false),
                                                                                new InstantCommand(()->m_dts.followJoystick())
                                                                            ),
                                                    // new MoveToScore(m_dts, m_otsc, 0, 0, false),
                                                    new ArmPneumaticCommand(m_aps, true)
                                                )
                                                
                                                // , 
                                                // // No Time to tune lining up for release and score
                                                // // release and score
                                                // new ClawPneumaticCommand(m_cps, true)
                                            
                                            );
        return s;
    }
}
