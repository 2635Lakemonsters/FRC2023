// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Pose {
    public Pose() {
      this.targetExtend = false;
      this.targetTheta = Constants.ARM_RETRACTED_LOWER_LIMIT;
    }
    
    /**
    @param targetExtend whether arm pneumatics are extended (true) or not (false)
    @param targetTheta angle of upper arm relative to lower arm (NOT floor)
    **/
    public Pose(boolean targetExtend, int targetTheta) {
      this.targetExtend = targetExtend;
      this.targetTheta = targetTheta;
    }
    public boolean targetExtend;
    public int targetTheta;
    
  };