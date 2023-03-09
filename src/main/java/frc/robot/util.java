// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ARM_STATE;
import frc.robot.Constants.ARM_TRANSITION;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public final class util {

    //
    // back:    167 - 209
    // forward: 210 - 273
    //
    // sweep:   40 deg sweep
    //
    // TODO: Need to measure on the robot.
    //
    // upper transition protection
    //       1) with lower arm in mid-transition pointing back, measure
    //            safeTopTransition1: 
    //              angle that the upper arm is at when the top of the arm wheel
    //              is perpendicularly above the lower pivot point of the lower arm.
    //              if the angle w.r.t. lower arm is greater than this angle, then 
    //              it is safe to transition between BM and FM
    //       2) with lower arm in mid-transition pointing forward, measure
    //            safeTopTransition2:
    //              similar to above
    //              if the arm angle is less than this angle, it is safe to transition
    //              pneumatics between BM and FM
    //
    // BM2BM
    // BM2BP requires upper transition protection
    // BM2FM requires upper transition protection 
    //       * actually could be:
    //         BM2BP
    //         BP2FM
    // BM2FP requires upper transition protection
    //       * actually could be:
    //         BM2FM
    //         FM2FP
    //
    // BP2BM requires upper transition protection
    // BP2BP
    // BP2FM requires lower tranistion protection 1
    //       * move arm to 210 prior to pneumatics move forward... 
    //       * 210 is a magic number for transition between BP2FM and FM2BP
    //         which is valid both forward and back regardless of transition
    // BP2FP requires lower tranistion protection 2
    //       * move arm to 273 or greater prior to moving pneumatics forward
    //       * NOTE: Likely taken care of by current state transition logic in the command.
    //
    // FM2BM requires upper transition protection
    //       * actually could be:
    //         FM2BP (set to 210 prior to moving pneumatics)
    //         BP2BM requires upper transition protection
    // FM2BP requires lower transition protection 1
    //       * move to 210 or greater prior to pneumatics moving back
    // FM2FM
    // FM2FP requires lower transition protection 2
    //       * pneumatics move back
    //       * move arm to 273 or greater prior to moving pneumatics forward
    //       * NOTE: Likely taken care of by current state transition logic in the command.
    //
    // FP2BM requires upper transition protection
    //       * actually could be:
    //         FP2FM
    //         FM2BM
    // FP2BP safe,no special protection required
    //       due to geometry, rotating back FP2BP is safe and will never cause a boundary violation
    // FP2FM requires lower transition protection 1
    //       * move pneumatics back
    //       * move arm to < 210 prior to moving pneumatics forward
    //       * NOTE: Likely taken care of by current state transition logic in the command.
    // FP2FP
    //
    // Transitioning betwen
    //-----
    // SCRATCH NOTES... DELETE once we figure it out.
    //-----
    // Need to define 
    // BM2BP
    //   safeTransitionBM2FM = 167 - sweep
    //   safeTransitionFM2BM = 209 + sweep???  This is too conservative... makes it impossible to work.
    //
    // Rules:
    //   Lower Arm has ~40 deg sweep, 
    //     BM2FM check if angle w.r.t. lower arm is > (167 - sweepAngle) if so, move arm to (167Do not do transition BM2FM unl
    //-----

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

    public static boolean isRed(int tagNum) {
        switch (tagNum)
        {
            case Constants.APRIL_TAG_ID_RedLeft:
            case Constants.APRIL_TAG_ID_RedMiddle:
            case Constants.APRIL_TAG_ID_RedRight:
            case Constants.APRIL_TAG_ID_RedSubstation:
                return true;

            default:
                return false;
        }
    }

    
    public static ARM_STATE getArmState(boolean bExtended, double theta) {
        if (bExtended) {
            double midPoint = (Constants.Hplus + Constants.Hminus) / 2.;
            if ((theta < Constants.Hplus) && (theta > Constants.Hminus)) {
                System.out.println("TODO: LOG ERROR STATE. Arm violating extended position robot bounds. util.getArmState() theta: "+ theta);
            }
            // return the current state
            if (theta >= midPoint) {
                return ARM_STATE.Fplus;
            } else if (theta < midPoint) {
                return ARM_STATE.Fminus;
            } else {
                // TODO: This should never happen. Log this error state
                System.out.println("TODO: LOG ERROR STATE. returning invalid arm state horizontal. util.getArmState() theta: "+ theta + " THIS SHOULD NEVER HAPPEN!!!!!");
                return ARM_STATE.InvalidHorz;
            }
        } else {
            double midPoint = (Constants.Vplus + Constants.Vminus) / 2.;
            if ((theta < Constants.Hplus) && (theta > Constants.Hminus)) {
                System.out.println("TODO: LOG ERROR STATE. Arm violating retracted position robot bounds. util.getArmState() theta: "+ theta);
            }
            if (theta >= midPoint) {
                return ARM_STATE.Bplus;
            } else if (theta < midPoint) {
                return ARM_STATE.Bminus;
            } else {
                // TODO: This should never happen. Log this error state
                System.out.println("TODO: LOG ERROR STATE. returning invalid arm state vertical. util.getArmState() theta: "+ theta + " THIS SHOULD NEVER HAPPEN!!!!!");
                return ARM_STATE.InvalidVert;
            }
        }
    }

    public static ARM_STATE getArmState() {
        // if (!RobotContainer.m_armMotorSubsystem.areWeThereYet()) {
        //     return ARM_STATE.Moving;
        // }
        boolean bExtended = RobotContainer.m_armPneumaticSubsystem.getIsExtended();
        double theta = RobotContainer.m_armMotorSubsystem.getTheta();
        return getArmState(bExtended, theta);
    }

    public static ARM_TRANSITION getTransition(boolean bExtended, double theta)
    {
        ARM_STATE state = getArmState(bExtended, theta);
        System.out.println(getArmState() + "->" + state);
        switch (state)
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

    public static boolean areWeRed() {
        return DriverStation.getAlliance() == DriverStation.Alliance.Red;
    }
}