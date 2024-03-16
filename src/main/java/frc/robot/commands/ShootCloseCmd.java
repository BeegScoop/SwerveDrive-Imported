// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HerderSubsystem;

public class ShootCloseCmd extends Command {
  private final HerderSubsystem herderSubsystem;
  private final ArmSubsystem armSubsystem;
  private final FlyWheelSubsystem flyWheelSubsystem;
  private final Timer timer;
  private boolean runOnce;
  /** Creates a new ShootCloseCmd. */
  public ShootCloseCmd(HerderSubsystem herderSubsystem, ArmSubsystem armSubsystem, FlyWheelSubsystem flyWheelSubsystem) {
    this.herderSubsystem = herderSubsystem;
    this.armSubsystem = armSubsystem;
    this.flyWheelSubsystem = flyWheelSubsystem;
    addRequirements(herderSubsystem, armSubsystem, flyWheelSubsystem);
    timer = new Timer();
    runOnce = true;
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
  public void execute(){

    if(timer.get()<0.17){
      if(runOnce){
        armSubsystem.setArmPosition(ArmConstants.kSpeakerCloseAngle);
        runOnce = false;
      }
      herderSubsystem.herderOut();
      flyWheelSubsystem.flyIn();
    }else if(timer.get()<1.5){
      herderSubsystem.herderStop();
      flyWheelSubsystem.flyOut();
              runOnce = true;

    }else if(timer.get()<2.5){
      herderSubsystem.herderIn();

    }else{
      herderSubsystem.herderStop();
      flyWheelSubsystem.flyStop();
      if(runOnce){
        armSubsystem.setArmPosition(ArmConstants.kHerdAngle);
        runOnce = false;
      }
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

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
