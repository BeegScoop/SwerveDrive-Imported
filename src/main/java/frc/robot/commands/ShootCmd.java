// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HerderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Timer;
import java.util.TimerTask;

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
    //backs the motor up for a certain time in order to give the flywheel time to spin up
    //edit delay for the time it takes for motor to spin up
    Timer timer = new Timer();
    herderSubsystem.herderOut();
    flyWheelSubsystem.flyOut();
    timer.schedule(new TimerTask() {
    @Override
    public void run() {
      herderSubsystem.herderStop();
    }//delay in ms
    }, 200);
    timer.schedule(new TimerTask() {
    @Override
    public void run() {
      herderSubsystem.herderIn();
    }//waits 5 seconds before shooting
    }, 5000);
  
      
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
