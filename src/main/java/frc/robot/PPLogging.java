// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class PPLogging {

    public static void logActiveTrajectory(PathPlannerTrajectory traj)
    {

    }

    public static void logTargetPose(Pose2d pose)
    {
        SmartDashboard.putNumber("PPSwerveControllerCommand/xTargetPose", pose.getX());
        SmartDashboard.putNumber("PPSwerveControllerCommand/yTargetPose", pose.getY());
        SmartDashboard.putNumber("PPSwerveControllerCommand/tTargetPose", pose.getRotation().getDegrees());

    }
    
    public static void logSetpoint(ChassisSpeeds speeds)
    {
        SmartDashboard.putNumber("PPSwerveControllerCommand/xSpeed", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("PPSwerveControllerCommand/ySpeed", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("PPSwerveControllerCommand/omega", Math.toDegrees(speeds.omegaRadiansPerSecond));
    }
    
    public static void logError(Translation2d translationError, Rotation2d rotationError)
    {
        SmartDashboard.putNumber("PPSwerveControllerCommand/xErrorMeters", translationError.getX());
        SmartDashboard.putNumber("PPSwerveControllerCommand/yErrorMeters", translationError.getY());
        SmartDashboard.putNumber(
            "PPSwerveControllerCommand/rotationErrorDegrees", rotationError.getDegrees());
    
    }
      
}
