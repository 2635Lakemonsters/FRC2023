// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ARM_STATE;
import frc.robot.Constants.ARM_TRANSITION;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import frc.robot.RobotContainer;

/** Add your docs here. */
public final class util {

    public static double inchesToMeters(double inches){
        return inches / 39.37;
    }
    public static double feetToMeters(double feet){
        return feet / 3.281;
    }
    public static double deadband(double input){
        return Math.abs(input) < 0.05? 0.0:input;
    }
    public static double metersToFeet(double meters){
        return meters * 3.281;
    }

    public static ARM_STATE getArmState(boolean bExtended, double theta) {
        if (!RobotContainer.m_armMotorSubsystem.areWeThereYet()) {
            return ARM_STATE.Moving;
        }
        if (bExtended) {
            if (theta > Constants.Hplus) {
                return ARM_STATE.Fplus;
            } else if (theta < Constants.Hminus) {
                return ARM_STATE.Fminus;
            } else {
                return ARM_STATE.InvalidHorz;
            }
        } else {
            if (theta > Constants.Vplus) {
                return ARM_STATE.Bplus;
            } else if (theta < Constants.Vminus) {
                return ARM_STATE.Bminus;
            } else {
                return ARM_STATE.InvalidVert;
            }
        }
    }

    public static ARM_STATE getArmState() {
        boolean bExtended = RobotContainer.m_armPneumaticSubsystem.getIsExtended();
        double theta = RobotContainer.m_armMotorSubsystem.getTheta();
        return getArmState(bExtended, theta);
    }

    public static ARM_TRANSITION getTransition(boolean bExtended, double theta)
    {
        switch (getArmState(bExtended, theta))
        {
            case Fplus:
                switch (getArmState())
                {
                    case Fplus:   return ARM_TRANSITION.FPlus2FPlus;
                    case Fminus:  return ARM_TRANSITION.FMinus2FPlus;
                    case Bplus:   return ARM_TRANSITION.BPlus2FPlus;
                    case Bminus:  return ARM_TRANSITION.BMinus2FPlus;
                    
                    default:      return ARM_TRANSITION.Illegal;
                }
            case Fminus:
                switch (getArmState())
                {
                    case Fplus:   return ARM_TRANSITION.FPlus2FMinus;
                    case Fminus:  return ARM_TRANSITION.FMinus2FMinus;
                    case Bplus:   return ARM_TRANSITION.BPlus2FMinus;
                    case Bminus:  return ARM_TRANSITION.BMinus2FMinus;

                    default:      return ARM_TRANSITION.Illegal;
                }
            case Bplus:
                switch (getArmState())
                {
                    case Fplus:   return ARM_TRANSITION.FPlus2BPlus;
                    case Fminus:  return ARM_TRANSITION.FMinus2BPlus;
                    case Bplus:   return ARM_TRANSITION.BPlus2BPlus;
                    case Bminus:  return ARM_TRANSITION.BMinus2BPlus;

                    default:      return ARM_TRANSITION.Illegal;
                }
            case Bminus:
                switch (getArmState())
                {
                    case Fplus:   return ARM_TRANSITION.FPlus2BMinus;
                    case Fminus:  return ARM_TRANSITION.FMinus2BMinus;
                    case Bplus:   return ARM_TRANSITION.BPlus2BMinus;
                    case Bminus:  return ARM_TRANSITION.BMinus2BMinus;

                    default:      return ARM_TRANSITION.Illegal;
                }

            default: return ARM_TRANSITION.Illegal;
        }
    }
}