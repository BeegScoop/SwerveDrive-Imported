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
import frc.robot.Constants.HerderConstants;

import java.net.CacheRequest;

import com.ctre.phoenix6.hardware.CANcoder;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HerderSubsystem extends SubsystemBase {
  /** Creates a new HerderSubsystem. */
  private CANSparkMax topHerderMotor;
  private CANSparkMax botHerderMotor;
  private RelativeEncoder topHerderEncoder;
  private RelativeEncoder botHerderEncoder;
  
  public HerderSubsystem() {
    topHerderMotor = new CANSparkMax(HerderConstants.kTopHerderMotorPort, MotorType.kBrushless);
    botHerderMotor = new CANSparkMax(HerderConstants.kBotHerderMotorPort, MotorType.kBrushless);
    topHerderEncoder = topHerderMotor.getEncoder();
    botHerderEncoder = botHerderMotor.getEncoder();
    topHerderEncoder.setPositionConversionFactor(HerderConstants.kHerderEncoderRot2Meter);
    topHerderEncoder.setVelocityConversionFactor(HerderConstants.kHerderEncoderRPM2MeterPerSec);
    botHerderEncoder.setPositionConversionFactor(HerderConstants.kHerderEncoderRot2Meter);
    botHerderEncoder.setVelocityConversionFactor(HerderConstants.kHerderEncoderRPM2MeterPerSec);
  }

  public void herderIn(){
    if(HerderConstants.kTopHerderMotorReversed){
      topHerderMotor.set(HerderConstants.kHerderMotorSpeed*(-1.0));
    }else{
      topHerderMotor.set(HerderConstants.kHerderMotorSpeed);
    }
    if(HerderConstants.kBotHerderMotorReversed){
      botHerderMotor.set(HerderConstants.kHerderMotorSpeed);
    }else{
      botHerderMotor.set(HerderConstants.kHerderMotorSpeed*(-1.0));
    }
  }
  public void herderOut(){
    if(HerderConstants.kTopHerderMotorReversed){
      topHerderMotor.set(HerderConstants.kHerderMotorSpeed);
    }else{
      topHerderMotor.set(HerderConstants.kHerderMotorSpeed*(-1.0));
    }
    if(HerderConstants.kBotHerderMotorReversed){
      botHerderMotor.set(HerderConstants.kHerderMotorSpeed*(-1.0));
    }else{
      botHerderMotor.set(HerderConstants.kHerderMotorSpeed);
    }
  }
  public void herderStop(){
    topHerderMotor.set(0);
    botHerderMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
