// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HerderSubsystem;

public class HerdCmd extends Command {
  /** Creates a new HerdCmd. */
  private final ArmSubsystem armSubsystem;
  private final HerderSubsystem herderSubsystem;
  public HerdCmd(ArmSubsystem armSubsystem, HerderSubsystem herderSubsystem) {
    this.armSubsystem = armSubsystem;
    this.herderSubsystem = herderSubsystem;
    addRequirements(armSubsystem, herderSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    herderSubsystem.herderIn();
    armSubsystem.setArmPosition(ArmConstants.kHerdAngle);
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
