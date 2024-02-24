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
  private CANSparkMax herderMotor;
  private RelativeEncoder herderEncoder;
  public HerderSubsystem() {
    herderMotor = new CANSparkMax(HerderConstants.kHerderMotorPort, MotorType.kBrushless);
    herderEncoder = herderMotor.getEncoder();
    herderEncoder.setPositionConversionFactor(HerderConstants.kHerderEncoderRot2Meter);
    herderEncoder.setVelocityConversionFactor(HerderConstants.kHerderEncoderRPM2MeterPerSec);
  }

  public void herderIn(){
    if(HerderConstants.kHerderMotorReversed){
      herderMotor.set(HerderConstants.kHerderMotorSpeed*(-1.0));
    }else{
      herderMotor.set(HerderConstants.kHerderMotorSpeed);
    }
  }
  public void herderOut(){
    if(HerderConstants.kHerderMotorReversed){
        herderMotor.set(HerderConstants.kHerderMotorSpeed);
    }else{
        herderMotor.set(HerderConstants.kHerderMotorSpeed*(-1.0));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
