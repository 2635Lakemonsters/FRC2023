// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmMotorSubsystem;

public class ManualArmMotorCommand extends CommandBase {

  ArmMotorSubsystem m_armMotorSubsystem;
  double m_poseTarget;

  /** Creates a new MannualArmMotorCommand. */
  public ManualArmMotorCommand(ArmMotorSubsystem ams) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armMotorSubsystem = ams;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // manual control of the upper arm with z axis slider
    double val = -RobotContainer.rightJoystick.getRawAxis(3);
    double angle = (val + 1.0) * 180.0;
    m_armMotorSubsystem.setPose(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
