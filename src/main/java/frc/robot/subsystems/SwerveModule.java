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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


import com.ctre.phoenix6.hardware.CANcoder;


public class SwerveModule {
    //da motors :)
private CANSparkMax driveMotor;
private CANSparkMax turningMotor;
//built in encoders
private RelativeEncoder driveEncoder;
private RelativeEncoder turningEncoder;
//to move angle motor
private PIDController turningPidController;
//absolute encoder so the wheel position can be kept constantly
//is only used at the very beginning to reset relative encoders
private CANcoder absoluteEncoder;
//
private boolean absoluteEncoderReversed;
//offset position
//used to compensate for encoder error
private double absoluteEncoderOffsetRad;
private int turningMotorId;
private int absoluteEncoderId;

public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
                    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
                    this.absoluteEncoderReversed = absoluteEncoderReversed;
                    
                    
                    this.absoluteEncoder = new CANcoder(absoluteEncoderId);

                   
                
                    
                    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
                    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
                    this.turningMotorId=turningMotorId;
                    this.absoluteEncoderId=absoluteEncoderId;
                    driveEncoder = driveMotor.getEncoder();
                    //device number and canbus need to be changed.
                    turningEncoder = turningMotor.getEncoder();
                 
                    
                   
                    //uses the gear ratio of drive motor to wheels and wheel diameter to make relative drive encoder match up with meters traveled by bot
                    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
                    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
                    //uses the gear ratio of turn motor to wheels to make relative turn encoder read as radians turned by the wheel
                    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
                    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
                    
                    //yea this a pid controller idk why the pid controller uses that turning value it just works so...
                    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
                    //turns PID controller into a circular system (tells it that -pi is right next to pi (think circle))
                    turningPidController.enableContinuousInput(-Math.PI,Math.PI);
                    //read the method
                    resetEncoders();
                    }
    //gets drive encoder position in meters
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }
    //gets turning encoder position in radians
    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }
    //gets that swerve module position idk
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(),new Rotation2d(getTurningPosition()));
    }
    //gets drive encoder velocity in m/s
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    //gets turning encoder velocity in rad/s
    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }
    //
    public double getAbsoluteEncoderRad(){
        //getAbsolutePosition returns value from -0.5 to 0.5 (think cicle again (-0.5 is right next to 0.5))
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        //boom angle is now in radians
        angle *= (2*Math.PI);
        //offsets the angle according to typed constants
        //pro tip: use Pheonix Tuner X to zero the absolute encoders manually instead
        //EVERY TIME THE ABSOLUTE ENCODERS ARE UNPLUGGED THEY NEED TO BE RE-ZEROED
        angle -= absoluteEncoderOffsetRad;
        //if they are reversed turn that john negative
        if(absoluteEncoderReversed){
            return angle*(-1.0);
        }else{
            return angle;
        }
    }
    public double getAbsoluteEncoderReading(){
        //literally just returns the method above this one
        //ik its basically useless but im not changing it now
        return getAbsoluteEncoderRad();
    }
    
    //read the function title
    public void resetEncoders(){
        //zeros drive encoder
        driveEncoder.setPosition(0);
        //sets the relative turning encoder to it's absolute encoder's value in radians 
        //(remember we converted absolute to radians by multiplying by 2pi and 
        //relative to radians by using our gear ratio to create a conversion factor)
        turningEncoder.setPosition(getAbsoluteEncoderRad());

        
    }
    //gets absolute position of absolute encoders on a range of -0.5 to 0.5 (raw data with no conversions)
    public double getAbsolutePos(){
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    } 
    //gets the state of the module 
    public SwerveModuleState getState(){
        //creates a current state from the current drive velocity and turning position
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));    
    }
    
    
    public void setDesiredState(SwerveModuleState state){
        //does not set a state if there is no speed in the new state
        //need to test to see if this makes it so it can coast
        if(Math.abs(state.speedMetersPerSecond)<0.001){
            stop();
            return;
        }
        //optimize makes it so that the wheel only has to turn a max of 90 degrees at a time
        //uses both directions of drive motor instead of just forward
        state = SwerveModuleState.optimize(state, getState().angle);
        //sets the driveMotor speed and scale down using physical max meters per second of the motor
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //sets the turningMotor position useing a pid controller with inputs of the current position of turning motor
        //combined with the desired position
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        // ???
        SmartDashboard.putString("Swerve[" + turningMotorId+"] state", state.toString());

    }
    //stop
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
