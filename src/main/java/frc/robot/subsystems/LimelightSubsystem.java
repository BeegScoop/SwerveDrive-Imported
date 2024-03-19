// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimeConstants;

///////////////////////////////////////////////////I DONT THINK WE NEED THIS SUBSYSTEM DUE TO LIMELIGHT HELPER\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
////////////////////////////////////////////////////////////////////DO NOT USE\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  private double x;
  private double y;
  private double area;
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("canMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
    LimelightHelpers.setStreamMode_Standard("limelight");
    

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  x = tx.getDouble(0.0);
  y = ty.getDouble(0.0);
  area = ta.getDouble(0.0);
  
  SmartDashboard.putNumber("limelightX", x);
  SmartDashboard.putNumber("limelightY", y);
  SmartDashboard.putNumber("limelightArea", area);
  SmartDashboard.putNumber("Distance from Tag", getHorizontalDistance());

  }
  public double getLimeX(){
    return x;
  }
  public double getLimeY(){
    return y;
  }
  public double getLimeArea(){
    return area;
  }
  public double getHorizontalDistance(){
    return ((LimeConstants.kSpeakerTagHieght-LimeConstants.kCameraHieght)/ //
    Math.tan((LimeConstants.kCameraAngle+LimelightHelpers.getTY("limelight"))*(Math.PI/180)))/12;
  }

  public void align(){
    
  }
}
