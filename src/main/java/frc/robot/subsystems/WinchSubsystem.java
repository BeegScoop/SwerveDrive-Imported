// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HerderConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.WinchConstants;


import java.net.CacheRequest;

import com.ctre.phoenix6.hardware.CANcoder;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchSubsystem extends SubsystemBase {
  /** Creates a new WinchSubsystem. */
  private CANSparkMax winchMotor;
  private final RelativeEncoder winchEncoder;

  private PIDController winchPidController;
  public WinchSubsystem() {
    winchMotor = new CANSparkMax(WinchConstants.kWinchMotorPort, MotorType.kBrushless);
    winchEncoder = winchMotor.getEncoder();
    winchEncoder.setPositionConversionFactor(WinchConstants.kWinchEncoderRot2Meter);
    winchEncoder.setVelocityConversionFactor(WinchConstants.kWinchEncoderRPM2MeterPerSec);
    winchPidController = new PIDController(WinchConstants.kPWinch, 0, 0);
    winchMotor.setSmartCurrentLimit(30);
  }
  public void extendLift(){
    if(WinchConstants.kWinchMotorReversed){
      winchMotor.set(WinchConstants.kWinchForwardSpeed*(-1.0));
    }else{
      winchMotor.set(WinchConstants.kWinchForwardSpeed);
    }
    
  }
  public void retractLift(){
    if(ArmConstants.kArmMotorReversed){
      winchMotor.set(WinchConstants.kWinchBackwardSpeed*(-1.0));
    }else{
      winchMotor.set(WinchConstants.kWinchBackwardSpeed);
    }
  }
  //I dont think this is going to be of use
  public void setLiftPosition(double posMeters){
    //does this need a reversal????
    //test
    winchMotor.set(winchPidController.calculate(winchEncoder.getPosition(), posMeters));
  }
  public void liftStop(){
    winchMotor.set(0);
  }
  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Relative Winch Encoder", winchEncoder.getPosition() );
    // This method will be called once per scheduler run
  }
}
