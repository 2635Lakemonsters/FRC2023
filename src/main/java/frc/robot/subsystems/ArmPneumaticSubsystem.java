// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

  import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
  import edu.wpi.first.wpilibj.Compressor;
  import edu.wpi.first.wpilibj.DoubleSolenoid;
  import edu.wpi.first.wpilibj.PneumaticsModuleType;
  import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
  import edu.wpi.first.wpilibj2.command.SubsystemBase;
  import frc.robot.Constants;

  public class ArmPneumaticSubsystem extends SubsystemBase {
    /** Creates a new CompressorSubsystem. */
    private DoubleSolenoid doubleSolenoid;

    public ArmPneumaticSubsystem() {
      // define the constants in the constants folder
      doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.EXTEND_CHANNEL, Constants.RETRACT_CHANNEL);

      doubleSolenoid.set(kOff);
    }
  
    public boolean isExtended() {
      if (doubleSolenoid.get() == kReverse) {
        return true;
      } else {
        return false;
      }
    
    }
    public void armExtend() {
	  	doubleSolenoid.set(kForward);

	  }
	  public void armRetract() {
	  	doubleSolenoid.set(kReverse);
	  }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}