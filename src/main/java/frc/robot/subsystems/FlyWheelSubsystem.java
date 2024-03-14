// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FlyConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


import com.ctre.phoenix6.hardware.CANcoder;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlyWheelSubsystem extends SubsystemBase {
  /** Creates a new HerderSubsystem. */
  private CANSparkMax rightFlyMotor;
  private CANSparkMax leftFlyMotor;
  private RelativeEncoder rightFlyEncoder;
  private RelativeEncoder leftFlyEncoder;
  
  public FlyWheelSubsystem() {
    rightFlyMotor = new CANSparkMax(FlyConstants.kRightFlyMotorPort,MotorType.kBrushless);
    leftFlyMotor = new CANSparkMax(FlyConstants.kLeftFlyMotorPort,MotorType.kBrushless);
    rightFlyEncoder = rightFlyMotor.getEncoder();
    leftFlyEncoder = leftFlyMotor.getEncoder();
    rightFlyEncoder.setPositionConversionFactor(FlyConstants.kFlyEncoderRot2Meter);
    leftFlyEncoder.setPositionConversionFactor(FlyConstants.kFlyEncoderRot2Meter);
    rightFlyEncoder.setVelocityConversionFactor(FlyConstants.kFlyEncoderRPM2MeterPerSec);
    leftFlyEncoder.setVelocityConversionFactor(FlyConstants.kFlyEncoderRPM2MeterPerSec);
  }
  public void flyIn(){
    if(FlyConstants.kLeftFlyMotorReversed){
      leftFlyMotor.set(FlyConstants.kFlyMotorSpeed*(-1.0));
    }else{
      leftFlyMotor.set(FlyConstants.kFlyMotorSpeed);
    }
    
    if(FlyConstants.kRightFlyMotorReversed){
      rightFlyMotor.set(FlyConstants.kFlyMotorSpeed*(-1.0));
    }else{
      rightFlyMotor.set(FlyConstants.kFlyMotorSpeed);
    }
    
  }
  public void flyOut(){
    if(FlyConstants.kLeftFlyMotorReversed){
      leftFlyMotor.set(FlyConstants.kFlyMotorSpeed);
    }else{
      leftFlyMotor.set(FlyConstants.kFlyMotorSpeed*(-1.0));
    }
    if(FlyConstants.kRightFlyMotorReversed){
      rightFlyMotor.set(FlyConstants.kFlyMotorSpeed);
    }else{
      rightFlyMotor.set(FlyConstants.kFlyMotorSpeed*(-1.0));
    }
    
    
  }
  public void flyStop(){
    leftFlyMotor.set(0);
    rightFlyMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Relative rightFly Encoder", rightFlyEncoder.getPosition() );
    SmartDashboard.putNumber("Relative leftFly Encoder", leftFlyEncoder.getPosition() );
    SmartDashboard.putNumber("Relative rightFly Encoder Velocity", rightFlyEncoder.getVelocity() );
    SmartDashboard.putNumber("Relative leftFly Encoder Velocity", leftFlyEncoder.getVelocity() );
  }
}
