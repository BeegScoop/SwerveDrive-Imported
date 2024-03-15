// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


import com.ctre.phoenix6.hardware.CANcoder;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private AbsoluteEncoder armAbsoluteEncoder;
  private PIDController armPidController;
  public ArmSubsystem() {
    
    armMotor = new CANSparkMax(ArmConstants.kArmMotorPort,MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();
    armAbsoluteEncoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    armEncoder.setPositionConversionFactor(ArmConstants.kArmEncoderRot2Rad);
    armEncoder.setVelocityConversionFactor(ArmConstants.kArmEncoderRPM2RadPerSec);
    //these dont do anything ig
    armAbsoluteEncoder.setPositionConversionFactor(ArmConstants.kArmAbsoluteRot2Rad);
    armAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.kArmAbsoluteRPM2RadPerSec);

    armPidController = new PIDController(ArmConstants.kPArm, 0, 0);
    armMotor.setSmartCurrentLimit(30);
    armEncoder.setPosition(armAbsoluteEncoder.getPosition());
    
    
  }
  public void turnArmForward(){
   
    if((armEncoder.getPosition())>ArmConstants.kRadLimitBot){
      if(ArmConstants.kArmMotorReversed){
        armMotor.set(ArmConstants.kForwardSpeed*(-1.0));
      }else{
        armMotor.set(ArmConstants.kForwardSpeed);
      }
    }else {
      armMotor.set(0);
    }
    
  }
  public void turnArmBackward(){
  

    if((armEncoder.getPosition())<ArmConstants.kRadLimitTop){
      if(ArmConstants.kArmMotorReversed){
        armMotor.set(ArmConstants.kBackwardSpeed*(-1.0));
      }else{
        armMotor.set(ArmConstants.kBackwardSpeed);
      }
    }else{
      armMotor.set(0);
    }
  }
  public void stopArm(){
    armMotor.set(0);
  }
  //input a radius relative to the starting position of the arm
  //BE CAREFUL: PID COULD BE SET IN WRONG DIRECTION SO START ARM IN THE MIDDLE AND BE READY TO STOP THAT JOHN
  public void setArmPosition(double posRad){
    //does this need a reversal????
    //test
    armMotor.set(armPidController.calculate(armEncoder.getPosition(), posRad));
  }
  public double getArmPosition(){
    return armEncoder.getPosition();

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Relative Arm Encoder", armEncoder.getPosition() );
        SmartDashboard.putNumber("Absolute Arm Encoder", armAbsoluteEncoder.getPosition());

  }


}
