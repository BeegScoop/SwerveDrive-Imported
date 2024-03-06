// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HerderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootCmd extends Command {
  /** Creates a new ShootCmd. */
  private final FlyWheelSubsystem flyWheelSubsystem;
  private final HerderSubsystem herderSubsystem;
  public ShootCmd(FlyWheelSubsystem flyWheelSubsystem, HerderSubsystem herderSubsystem ) {
    this.flyWheelSubsystem =flyWheelSubsystem;
    this.herderSubsystem = herderSubsystem;
    addRequirements(flyWheelSubsystem,herderSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flyWheelSubsystem.flyOut();;
    herderSubsystem.herderIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
