// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmMotorSubsystem extends SubsystemBase {

  public TalonFX armMotor = new TalonFX(Constants.TALON_CHANNEL);
  private  final double kFilterArm = 0.1;
  private  double armPOFiltered = 0;
  private  long loopCtr = 0;
  private PIDController pid = new PIDController(0.005, 0.0, 0.0);
  private  double theta;
  private  double m_poseTarget;
  
  /** Creates a new ArmMotorSubsystem. */
  public ArmMotorSubsystem() {
    pid.setTolerance(20);
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

    m_poseTarget = MathUtil.clamp(m_poseTarget, lowerLimit, upperLimit);

    // manual control of the upper arm with z axis slider
    // double val = -RobotContainer.rightJoystick.getRawAxis(3);
    // double angle = (val+1.0)*180.0;
    // m_poseTarget = MathUtil.clamp(angle, lowerLimit, upperLimit);


    theta = 360.0 * (RobotContainer.encoder.getValue() - Constants.ARM_ENCODER_OFFSET) / 4096.0;
    while (theta < 0)
      theta += 360.0;
    theta %= 360.0;

    double fPO = (theta + alpha - 90.0);
    while (fPO < 0)
      fPO += 360.0;
    fPO %= 360.0;

    final double gain = Constants.ARM_MOTOR_FF_GAIN;
    double ffMotorPower = gain * Math.sin(Math.toRadians(fPO));

    double fbMotorPower = MathUtil.clamp(pid.calculate(theta, m_poseTarget), Constants.FB_LOWER_LIMIT, Constants.FB_UPPER_LIMIT);

    // armPOFiltered = kFilterArm * motorPower + (1.0 - kFilterArm) * armPOFiltered;
    // System.out.println(armPOFiltered);
    // if (loopCtr % 50 == 0) {
    //   System.out.print("FF Motor Power = " + ffMotorPower);
    //   System.out.println("   FB Motor Power = " + fbMotorPower);
    //   System.out.println("fPO = " + fPO + "   Theta = " + theta + "   Alpha = " + alpha + "   Raw = " + RobotContainer.encoder.getValue());
    // }
    armMotor.set(ControlMode.PercentOutput, fbMotorPower - ffMotorPower);
  }

  public double getTheta() { // arm angle with respect to the lower arm
    return theta;
  }

  public void setPose(double poseTarget) {
    m_poseTarget = poseTarget;
  }

  public boolean areWeThereYet() {
    return pid.atSetpoint();
  }
 }
