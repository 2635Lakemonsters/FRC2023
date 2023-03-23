// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmMotorSubsystem extends SubsystemBase {

  public TalonFX armMotor = new TalonFX(Constants.TALON_CHANNEL);
  // private  final double kFilterArm = 0.1;
  // private  double armPOFiltered = 0;
  private  long loopCtr = 0;
  private PIDController pid = new PIDController(0.012, 0.0, 0.001); //added a little bit of kd to damp out velocity
  private  double theta;
  private  double m_poseTarget = Constants.TOP_SCORING_ANGLE - 10;
  private double fPO;
  private ClawPneumaticSubsystem cns;
  
  /** Creates a new ArmMotorSubsystem. */
  public ArmMotorSubsystem(ClawPneumaticSubsystem cns) {
    this.cns = cns;
    pid.setTolerance(10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    loopCtr++;
    double upperLimit, lowerLimit, alpha;

    if (RobotContainer.m_armPneumaticSubsystem.getIsExtended()) {
      alpha = Constants.ARM_EXTENDED_ALPHA;
      lowerLimit = Constants.ARM_EXTENDED_LOWER_LIMIT;
      upperLimit = Constants.ARM_EXTENDED_UPPER_LIMIT;
    } else {
      alpha = Constants.ARM_RETRACTED_ALPHA;
      lowerLimit = Constants.ARM_RETRACTED_LOWER_LIMIT;
      upperLimit = Constants.ARM_RETRACTED_UPPER_LIMIT;
    }
    // Arm pose trim override
    Joystick hatJoystickTrimRotationArm = (Constants.HAT_JOYSTICK_TRIM_ROTATION_ARM == Constants.LEFT_JOYSTICK_CHANNEL)
      ? RobotContainer.leftJoystick
      : RobotContainer.rightJoystick;
    if(hatJoystickTrimRotationArm.getPOV()==Constants.HAT_POV_ARM_UP){
      m_poseTarget += Constants.HAT_POSE_TARGET_PER_TIME_STEP;
    }
    if(hatJoystickTrimRotationArm.getPOV()==Constants.HAT_POV_ARM_DOWN){
      m_poseTarget += Constants.HAT_POSE_TARGET_PER_TIME_STEP*-1.0;
    }
    m_poseTarget = MathUtil.clamp(m_poseTarget, lowerLimit, upperLimit);

    // manual control of the upper arm with z axis slider
    // double val = -RobotContainer.rightJoystick.getRawAxis(3);
    // double angle = (val+1.0)*180.0;
    // m_poseTarget = MathUtil.clamp(angle, lowerLimit, upperLimit);


    theta = 360.0 * (RobotContainer.encoder.getValue() - Constants.ARM_ENCODER_OFFSET) / 4096.0;
    theta %= 360.0;
    if (theta < 0)
      theta += 360.0;

    fPO = (theta + alpha - 90.0);
    fPO %= 360.0;
    if (fPO < 0)
      fPO += 360.0;

    final double gain = Constants.ARM_MOTOR_FF_GAIN;
    double ffMotorPower = gain * Math.sin(Math.toRadians(fPO));

    double lowerLimitFB = cns.getIsClosed() ? Constants.FB_LOWER_LIMIT_CLOSED : Constants.FB_LOWER_LIMIT_OPEN;
    double upperLimitFB = cns.getIsClosed() ? Constants.FB_UPPER_LIMIT_CLOSED : Constants.FB_UPPER_LIMIT_OPEN;
    double fbMotorPower = MathUtil.clamp(pid.calculate(theta, m_poseTarget), lowerLimitFB, upperLimitFB);

    // armPOFiltered = kFilterArm * motorPower + (1.0 - kFilterArm) * armPOFiltered;
    // System.out.println(armPOFiltered);
    if (loopCtr % 50 == 0) {
    //   System.out.println("theta=" + theta + "   Target = " + m_poseTarget);
    //   System.out.print("FF Motor Power = " + ffMotorPower);
    //   System.out.println("   FB Motor Power = " + fbMotorPower);
    //   System.out.println("fPO = " + fPO + "   Theta = " + theta + "   Alpha = " + alpha + "   Raw = " + RobotContainer.encoder.getValue());
    }
    double motorPower = fbMotorPower - ffMotorPower;
    armMotor.set(ControlMode.PercentOutput, motorPower);
    // SmartDashboard.putNumber("arm motor power", motorPower);
    // SmartDashboard.putNumber("arm ff motorpower", ffMotorPower);
    // SmartDashboard.putNumber("arm fb", fbMotorPower);
    // putToSDB();
  }

  public void putToSDB() {
    SmartDashboard.putNumber("theta", theta);
    SmartDashboard.putNumber("fPO", fPO);
    SmartDashboard.putNumber("Raw", RobotContainer.encoder.getValue());
    SmartDashboard.putNumber("pneumatic state", (RobotContainer.m_armPneumaticSubsystem.getIsExtended() == true ? 1 : 0));
  }

  public double getTheta() { // arm angle with respect to the lower arm
    return theta;
  }

  public void setPose(double poseTarget) {
    m_poseTarget = poseTarget;
    SmartDashboard.putNumber("Pose Target", poseTarget);
  }

  public boolean areWeThereYet() {
    double sp = pid.getSetpoint();
    boolean atSP = pid.atSetpoint();
    return sp == 0.0 || atSP;
  }
 }
