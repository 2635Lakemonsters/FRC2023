// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.legacymath2910.MathUtils;
import frc.robot.models.VisionObject;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import frc.robot.subsystems.ClawPneumaticSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class AlignGripperToObjectCommand extends CommandBase {
  DrivetrainSubsystem m_drivetrainSubsystem;
  ObjectTrackerSubsystem m_objectTrackerSubsystem;
  ArmPneumaticSubsystem m_armPneumaticSubsystem;
  ClawPneumaticSubsystem m_clawPneumaticSubsystem;

  PIDController strafeController;
  PIDController forwardController; 

  boolean allDone = false;
  boolean objectLost = false;
  Timer lostTime = new Timer();

  /** Creates a new AlignGripperToObjectCommand. 
   * Once arm is in position, this command centers/aligns the bot for the gripper to close around
   * 
   * 1) make sure gripper is open at start (initialize) use the armpneumaticcommand OR armpneumatic subsystem
   * 2) make sure arm is in right configuration
   * 3) alignment (execute) - call drivetrainSubsystem methods? and/or move the arm to align to the object
   * 4) Align claw to the object using armpneumaticsubsystem and clawpneumaticsubsystem 
  */
  public AlignGripperToObjectCommand(DrivetrainSubsystem ds, ObjectTrackerSubsystem ots, ArmPneumaticSubsystem aps, ClawPneumaticSubsystem cps) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = ds;
    m_objectTrackerSubsystem = ots;
    m_armPneumaticSubsystem = aps;
    m_clawPneumaticSubsystem = cps;

    addRequirements(ds, ots, aps, cps);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    allDone = false;
    objectLost = false;
    m_clawPneumaticSubsystem.grabberOpen();
    m_objectTrackerSubsystem.data();

    strafeController = new PIDController(2.0, 0.0, 0.0); // TODO update constants
    forwardController = new PIDController(1.8, 0.00, 0.0); // TODO update constants   
    System.out.println("Strafe Tol:" + strafeController.getPositionTolerance() + "Forward Tol:" + forwardController.getPositionTolerance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_objectTrackerSubsystem.data();
    VisionObject closestObject = m_objectTrackerSubsystem.getClosestObject();
    // If we don't see an object for 1/4 second, give up.  Otherwise, be a little patient
    if (closestObject == null)
    {
      if (objectLost)
      {
        if (lostTime.hasElapsed(0.25)) 
        {
          allDone = true;
        }
      }
      else
      {
        lostTime.restart();
        objectLost = true;
        m_drivetrainSubsystem.drive(0, 0, 0, false);
      }
      return;
    }
    double strafe = MathUtils.clamp(-strafeController.calculate(closestObject.x, 0.5), -0.2, 0.2);
    double forward = MathUtils.clamp(-forwardController.calculate(closestObject.y, 0.5), -0.2, 0.2);
    // SmartDashboard.putNumber("strafe", strafe);
    // SmartDashboard.putNumber("forward", forward);
    // System.out.println("strafe: " + strafe + "   forward: " + forward);
    m_drivetrainSubsystem.drive(forward, strafe, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean sa = strafeController.atSetpoint();
    boolean fa = forwardController.atSetpoint();
    // if (sa)
    //   System.out.println("Strafe aligned");
    // if (fa)
    //   System.out.println("Forward aligned");
    boolean finished = allDone || (sa && fa);
    // if (finished)
    //   System.out.println("Align complete");
    return finished;
  }
}
