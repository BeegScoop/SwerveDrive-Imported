// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HerderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class ShootCmd extends Command {
  /** Creates a new ShootCmd. */
  private Timer timer;
  private boolean done = false;
  private final FlyWheelSubsystem flyWheelSubsystem;
  private final HerderSubsystem herderSubsystem;
  public ShootCmd(FlyWheelSubsystem flyWheelSubsystem, HerderSubsystem herderSubsystem ) {
    this.flyWheelSubsystem =flyWheelSubsystem;
    this.herderSubsystem = herderSubsystem;
    addRequirements(flyWheelSubsystem,herderSubsystem);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
     timer.reset();
     done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //backs the motor up for a certain time in order to give the flywheel time to spin up
    //edit delay for the time it takes for motor to spin up
   
    
    if(timer.get()>4){
      herderSubsystem.herderStop();
      flyWheelSubsystem.flyStop();
      done = true;
    }else if(timer.get()>2){
       herderSubsystem.herderIn();
    }else if(timer.get()>0.15){
      herderSubsystem.herderStop();
      flyWheelSubsystem.flyOut();
    }else{
    herderSubsystem.herderOut();
    flyWheelSubsystem.flyIn();
    }
    
    
  
  
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    herderSubsystem.herderStop();
    flyWheelSubsystem.flyStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(done){
      return true;
    }else{
    return false;
    }
  }
}
