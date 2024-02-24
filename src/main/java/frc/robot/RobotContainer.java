// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ResetGyroCmd;
import frc.robot.commands.SwerveJoystickCmd;


import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  //XBOX CONTROLLER ON PORT 0 AND JOYSTICK ON PORT 1
  //Press "X" on the xbox controller to toggle between
  private final Joystick driverJoystickOne = new Joystick(OIConstants.kDriverControllerOnePort);
    private final Joystick driverJoystickTwo = new Joystick(OIConstants.kDriverControllerTwoPort);
  //not used
  private final SendableChooser<String> m_chooser;
  public static final String xboxTxt = "Xbox";
  public static final String joystickTxt = "Joysitck";
  

  public RobotContainer() {
    m_chooser = new SendableChooser<>();
    m_chooser.addOption(xboxTxt, xboxTxt);
    m_chooser.addOption( joystickTxt,  joystickTxt);
    
      //sets default command to joystick with feeders from controller
      swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
              swerveSubsystem,
              () -> -driverJoystickOne.getRawAxis(OIConstants.kDriverYAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kDriverXAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kDriverRotAxisXbox),
              () -> !driverJoystickOne.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

      configureButtonBindings();
  }

  private void configureButtonBindings() {
    //resets the gyro mid drive
    new JoystickButton(driverJoystickOne, 2).onTrue(new ResetGyroCmd(swerveSubsystem)); 
    //switches over to joystick using x button toggle
    new JoystickButton(driverJoystickOne, 3).toggleOnTrue(new SwerveJoystickCmd(
              swerveSubsystem,
              () -> -driverJoystickTwo.getRawAxis(OIConstants.kDriverYAxis),
              () -> driverJoystickTwo.getRawAxis(OIConstants.kDriverXAxis),
              () -> driverJoystickTwo.getRawAxis(OIConstants.kDriverRotAxisJoystick),
              () -> !driverJoystickTwo.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); 


  }
  
  public Command getAutonomousCommand() {
    //1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(//
    AutoConstants.kMaxSpeedMetersPerSecond,//
    AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared)//
    .setKinematics(DriveConstants.kDriveKinematics);
    //2. Generate trajectory
    //maybe make more Autocommands that can pass in these values/cords
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(//
        //x is forward/backward movement and y is left/right movement
        //coordinates are in meters i think???
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
                new Translation2d(0.5, 0),
                new Translation2d(0.5, -0.5)),
        new Pose2d(1, -0.5, Rotation2d.fromDegrees(180)),
        trajectoryConfig);
    //3. Define PID controllers for tracking trajectory
    //check out these autoconstants
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController,0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    HolonomicDriveController holoController = new HolonomicDriveController(xController, yController, thetaController);
    //4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(//
    trajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    holoController, 
    swerveSubsystem::setModuleStates, 
    swerveSubsystem);
    //5. Add some init qand wrap-up, and return everything
    return new SequentialCommandGroup(
      //resets odometer so that "even if the robot does not start on the initial point of our trajectory, it will move that trajectory to the current location"
      new InstantCommand(()-> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),//
      swerveControllerCommand,//
      new InstantCommand(()-> swerveSubsystem.stopModules())//
    );
  }
}
