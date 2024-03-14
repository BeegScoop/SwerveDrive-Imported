// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HerderSubsystem;

public class HerderInCmd extends Command {
  /** Creates a new HerderInCmd. */
  private final HerderSubsystem herderSubsystem;
  public HerderInCmd(HerderSubsystem herderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.herderSubsystem = herderSubsystem;
    addRequirements(herderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    herderSubsystem.herderIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    herderSubsystem.herderStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
