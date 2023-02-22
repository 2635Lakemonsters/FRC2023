// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmMotorSubsystem extends SubsystemBase {

  public TalonFX armMotor = new TalonFX(Constants.TALON_CHANNEL);
  private static final double kFilterArm = 0.1;
  private static double armPOFiltered = 0;
  private static long loopCtr = 0;
  PIDController pid = new PIDController(0.005, 0.0, 0.0);
  
  /** Creates a new ArmMotorSubsystem. */
  public ArmMotorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double calculatePower(double targetPose) { // target encoder pose
    double currentPose = RobotContainer.encoder.getVoltage(); // Get current encoder pose 
    double delta = targetPose - currentPose;

    final double gain = 2.0;
    final double range = 0.15;
    double motorPower = gain * delta;
    motorPower = Math.min(Math.max(motorPower, -range), range);

    return motorPower;
  }
  
  public void setMotorPower(double motorPower) {
    /*Set Voltage in this method*/
    armMotor.set(ControlMode.PercentOutput, motorPower);
  }

  public void setPose(double poseTarget) {
    loopCtr++;
    double upperLimit, lowerLimit, alpha;

    // TODO: move magic #'s to Constants

    if (ArmPneumaticSubsystem.getIsExtended()) {
      alpha = Constants.ARM_EXTENDED_ALPHA;
      lowerLimit = Constants.ARM_EXTENDED_LOWER_LIMIT;
      upperLimit = Constants.ARM_EXTENDED_UPPER_LIMIT;
    } else {
      alpha = Constants.ARM_RETRACTED_ALPHA;
      lowerLimit = Constants.ARM_RETRACTED_LOWER_LIMIT;
      upperLimit = Constants.ARM_RETRACTED_UPPER_LIMIT;
    }

    poseTarget = MathUtil.clamp(poseTarget, lowerLimit, upperLimit);

    double theta = 360.0 * (RobotContainer.encoder.getValue() - Constants.ARM_ENCODER_OFFSET) / 4096.0;
    while (theta < 0)
      theta += 360.0;
    theta %= 360.0;

    double fPO = (theta + alpha - 90.0);
    while (fPO < 0)
      fPO += 360.0;
    fPO %= 360.0;

    final double gain = Constants.ARM_MOTOR_GAIN;
    double ffMotorPower = gain * Math.sin(Math.toRadians(fPO));

    double fbMotorPower = MathUtil.clamp(pid.calculate(theta, poseTarget), Constants.FB_LOWER_LIMIT, Constants.FB_UPPER_LIMIT);

    // armPOFiltered = kFilterArm * motorPower + (1.0 - kFilterArm) * armPOFiltered;
    // System.out.println(armPOFiltered);
    if (loopCtr % 50 == 0) {
      System.out.print("FF Motor Power = " + ffMotorPower);
      System.out.println("   FB Motor Power = " + fbMotorPower);
      System.out.println("fPO = " + fPO + "   Theta = " + theta + "   Alpha = " + alpha + "   Raw = " + RobotContainer.encoder.getValue());
    }

    // armMotor.set(ControlMode.PercentOutput, fbMotorPower - ffMotorPower);
  }
 }
