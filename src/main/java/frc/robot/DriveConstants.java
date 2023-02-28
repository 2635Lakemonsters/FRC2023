// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public final class DriveConstants {
    // public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxSpeed = 0.5;
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  
    public static final double m_drivetrainWheelbaseWidth = 18.5 / Constants.INCHES_PER_METER;
    public static final double m_drivetrainWheelbaseLength = 28.5 / Constants.INCHES_PER_METER;

    public static final Translation2d m_frontLeftLocation = 
            new Translation2d(m_drivetrainWheelbaseWidth/2, m_drivetrainWheelbaseLength/2);
    public static final Translation2d m_frontRightLocation = 
            new Translation2d(m_drivetrainWheelbaseWidth/2, -m_drivetrainWheelbaseLength/2);
    public static final Translation2d m_backLeftLocation = 
            new Translation2d(-m_drivetrainWheelbaseWidth/2, m_drivetrainWheelbaseLength/2);
    public static final Translation2d m_backRightLocation = 
            new Translation2d(-m_drivetrainWheelbaseWidth/2, -m_drivetrainWheelbaseLength/2);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      m_frontLeftLocation,
      m_frontRightLocation, 
      m_backLeftLocation, 
      m_backRightLocation);
    
      public static final double kMaxSpeedMetersPerSecond = 3;
}
