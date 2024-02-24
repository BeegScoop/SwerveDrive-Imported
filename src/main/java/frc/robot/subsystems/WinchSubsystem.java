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
  private RelativeEncoder winchEncoder;
  private PIDController winchPidController;
  public WinchSubsystem() {
    winchMotor = new CANSparkMax(WinchConstants.kWinchMotorPort, MotorType.kBrushless);
    winchEncoder = winchMotor.getEncoder();
    winchEncoder.setPositionConversionFactor(WinchConstants.kWinchEncoderRot2Meter);
    winchEncoder.setVelocityConversionFactor(WinchConstants.kWinchEncoderRPM2MeterPerSec);
    winchPidController = new PIDController(WinchConstants.kPWinch, 0, 0);
  }
  public void turnArmForward(){
    if(WinchConstants.kWinchMotorReversed){
      winchMotor.set(WinchConstants.kWinchForwardSpeed*(-1.0));
    }else{
      winchMotor.set(WinchConstants.kWinchForwardSpeed);
    }
    
  }
  public void turnArmBackward(){
    if(ArmConstants.kArmMotorReversed){
      winchMotor.set(WinchConstants.kWinchBackwardSpeed*(-1.0));
    }else{
      winchMotor.set(WinchConstants.kWinchBackwardSpeed);
    }
  }
  public void setArmPosition(double posRad){
    //does this need a reversal????
    //test
    winchMotor.set(winchPidController.calculate(winchEncoder.getPosition(), posRad));
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Relative Winch Encoder", winchEncoder.getPosition() );
    // This method will be called once per scheduler run
  }
}