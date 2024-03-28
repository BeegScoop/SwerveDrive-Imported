// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HerderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ShootCmd extends Command {
  /** Creates a new ShootCmd. */
  private Timer timer;
  
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
     
     timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //backs the motor up for a certain time in order to give the flywheel time to spin up
    //edit delay for the time it takes for motor to spin up
    SmartDashboard.putNumber("timer", timer.get());

    if(timer.get()<0.10){
      herderSubsystem.herderOut();
      flyWheelSubsystem.flyIn();
    }else if(timer.get()<1.5){
      herderSubsystem.herderStop();
      flyWheelSubsystem.flyOut();
    }else if(timer.get()<2.5){
      herderSubsystem.herderIn();

    }else{
      herderSubsystem.herderStop();
      flyWheelSubsystem.flyStop();
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
    if(timer.get()>2.5){
      return true;
    }else{
    return false;
    }
  }
}
