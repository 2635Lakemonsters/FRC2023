// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClawPneumaticSubsystem extends SubsystemBase {
  /** Creates a new CompressorSubsystem. */
  private DoubleSolenoid doubleSolenoid;

  public ClawPneumaticSubsystem() {
    // define the constants in the constants folder
    doubleSolenoid = RobotContainer.m_pneumaticHub.makeDoubleSolenoid(Constants.CLOSE_CHANNEL, Constants.OPEN_CHANNEL);

    doubleSolenoid.set(kOff);
  }
  
  public boolean isClosed() {
    if (doubleSolenoid.get() == kReverse) {
      return true;
    } else {
      return false;
    }
    
  }
  public void grabberOpen() {
		doubleSolenoid.set(kForward);

	}
	public void grabberClose() {
		doubleSolenoid.set(kReverse);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}