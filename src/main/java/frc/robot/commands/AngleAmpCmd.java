// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class AngleAmpCmd extends Command {
  /** Creates a new AngleAmpCmd. */
  private final ArmSubsystem armSubsystem;
  public AngleAmpCmd(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setArmPosition(ArmConstants.kAmpAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(armSubsystem.getArmPosition()<ArmConstants.kAmpAngle+0.05&&armSubsystem.getArmPosition()>ArmConstants.kAmpAngle-0.1){
      return true;
    }else{
      return false;
    }
    
  }
}
