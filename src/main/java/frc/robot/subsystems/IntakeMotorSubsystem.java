// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMotorSubsystem extends SubsystemBase {
  /** Creates a new IntakeMotorSubsystem. */
  private CANSparkMax m_rightMotor;
  private CANSparkMax m_leftMotor;
  private int currentMotorSpeed = 0;
  private boolean isIdleModeActive = false;
  private double idleOnTime = 1.0;//Calibrate Idle on and off times
  private double idleOffTime = 3.0;
  public Constants.GRIPPER_MOTOR_STATE gripperMotorState = Constants.GRIPPER_MOTOR_STATE.Off;
  public IntakeMotorSubsystem() {
    m_rightMotor = new CANSparkMax(0, MotorType.kBrushless);//Fill out IDs  
    m_leftMotor = new CANSparkMax(1, MotorType.kBrushless);//Wrong Ids


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //When isIdleModeActive is true:
    // run setMotorPoweridleOn for 1 second
    // run set motor poweridleoff for 3 seconds
   
    // if(isIdleModeActive){
    //   setMotorPowerIdleOn();
    // }
    // else{
    //   setMotorPowerOff();
    // }
    m_rightMotor.set(currentMotorSpeed);
    m_leftMotor.follow(m_rightMotor, false);//CHeck if is actually inverted

  }

  public void setMotorPower(int newMotorSpeed){
    currentMotorSpeed = newMotorSpeed;
  }

  public void setMotorPowerOff(){
    gripperMotorState = Constants.GRIPPER_MOTOR_STATE.Off;
    currentMotorSpeed = 0;
  }

  public void setMotorPowerIdleOn(){
    currentMotorSpeed = 1;//TODO calibrate motor speeds
    gripperMotorState = Constants.GRIPPER_MOTOR_STATE.IdleOn;

  }
  
  public void setMotorPowerIntakeCube(){
    currentMotorSpeed = 2;
    gripperMotorState = Constants.GRIPPER_MOTOR_STATE.IntakeCube;

  }
  
  public void setMotorPowerIntakeCone(){
    currentMotorSpeed = 2;
    gripperMotorState = Constants.GRIPPER_MOTOR_STATE.IntakeCone;

  }
  
  public void setMotorPowerOutakeCube(){
    currentMotorSpeed = 0;
    gripperMotorState = Constants.GRIPPER_MOTOR_STATE.OuttakeCube;

  }
  
  public void setMotorPowerOutakeConeEject(){
    currentMotorSpeed = 2;
    gripperMotorState = Constants.GRIPPER_MOTOR_STATE.OuttakeConeEject;

  }
  public void setMotorPowerOutakeConeDrop(){
    currentMotorSpeed = 0;
    gripperMotorState = Constants.GRIPPER_MOTOR_STATE.OuttakeConeDrop;

  }
}

