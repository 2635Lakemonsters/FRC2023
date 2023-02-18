// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

  import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
  import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
  import frc.robot.Constants;
import frc.robot.RobotContainer;

  public class ArmPneumaticSubsystem extends SubsystemBase {
    /** Creates a new CompressorSubsystem. */
    private DoubleSolenoid doubleSolenoid;

    public ArmPneumaticSubsystem() {
      // define the constants in the constants folder
      doubleSolenoid = RobotContainer.m_pneumaticHub.makeDoubleSolenoid(Constants.EXTEND_CHANNEL, Constants.RETRACT_CHANNEL);

      doubleSolenoid.set(kOff);
    }
  
    public boolean isExtended() {
      if (doubleSolenoid.get() == Value.kReverse) {
        return true;
      } else {
        return false;
      }
    
    }
    public void armExtend() {
	  	doubleSolenoid.set(Value.kForward);

	  }
	  public void armRetract() {
	  	doubleSolenoid.set(Value.kReverse);
	  }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}