package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
//connect to the rio
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
                    
                    //cancoder as absolute;
                    this.absoluteEncoder = new CANcoder(absoluteEncoderId);
                    //changing cancoder from a default range of (0,360) to (-180, 180) cuz thats how tf it works
                    //NOPE
                    
                    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
                    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
                    this.turningMotorId=turningMotorId;
                    this.absoluteEncoderId=absoluteEncoderId;
                    driveEncoder = driveMotor.getEncoder();
                    //device number and canbus need to be changed.
                    turningEncoder = turningMotor.getEncoder();
                 
                    
                    //Cancoder version of this???
                    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
                    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
                    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
                    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
                    

                    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
                    //turns PID controller into a circular system (tells it that -180 is right next to 179 (-180 and 180 cant both exist))
                    turningPidController.enableContinuousInput(-Math.PI,Math.PI);

                    resetEncoders();
                    }
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }
    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }
    public double getAbsoluteEncoderRad(){
        //how many percent of a full rotation it is currently reading
        //CHECK IF RETURNS DEGREES
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        angle *= (2*Math.PI);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed? -1.0 : 1.0);
    }
    public double getAbsoluteEncoderReading(){
        // return "Deg: " +absoluteEncoder.getAbsolutePosition()+ " Rad: "+getAbsoluteEncoderRad();
        return getAbsoluteEncoderRad();
    }
    
    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());

        
    }
    public double getAbsolutePos(){
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    } 
    
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));    
    }
    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond)<0.001){
            stop();
            return;

        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        // ???
        SmartDashboard.putString("Swerve[" + turningMotorId+"] state", state.toString());

        // SmartDashboard.putString("AbsoluteEncoder["+absoluteEncoderId+"]", "Rad:"+getAbsoluteEncoderRad() +", Deg:"+absoluteEncoder.getAbsolutePosition() );
    }
    
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
