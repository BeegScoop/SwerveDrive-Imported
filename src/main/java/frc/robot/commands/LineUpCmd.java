// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HerderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeConstants;


///IDEA: Use PID controller to take in tx input from limelight to twist to desired position
//use set state as in joystick commands
public class LineUpCmd extends Command {
  /** Creates a new LineUpAndShootCmd. */
  private final SwerveSubsystem swerveSubsystem;
  

  private final ProfiledPIDController twistController;
  public LineUpCmd(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    twistController = new ProfiledPIDController(
        LimeConstants.kPTwistController,0, LimeConstants.kDTwistController, LimeConstants.kTwistControllerConstraints);
    twistController.setTolerance(Math.PI/180);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double xSpeed = 0;
    double ySpeed = 0;
    double turningSpeed = -1.0*twistController.calculate(
    (LimelightHelpers.getTX("limelight")/180)*Math.PI,
    0
    );
    ChassisSpeeds chassisSpeeds;
    // Relative to field
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
                
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
    

    //add launching portion
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(LimelightHelpers.getTX("limelight")>1||LimelightHelpers.getTX("limelight")<-1){
      swerveSubsystem.stopModules();
      return true;
    }else{
      return false;
    }
  }
}
