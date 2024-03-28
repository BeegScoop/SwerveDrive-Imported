// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AngleAmpCmd;
import frc.robot.commands.AngleCloseSpeakerCmd;
import frc.robot.commands.AngleFarSpeakerCmd;
import frc.robot.commands.AngleHerdCmd;
import frc.robot.commands.ArmBwdCmd;
import frc.robot.commands.ArmFwdCmd;
import frc.robot.commands.FlyWheelInCmd;
import frc.robot.commands.FlyWheelOutCmd;
import frc.robot.commands.HerderInCmd;
import frc.robot.commands.HerderOutCmd;
import frc.robot.commands.HoldArmCmd;
import frc.robot.commands.LineUpCmd;
import frc.robot.commands.ResetGyroCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.FlyWheelInCmd;
import frc.robot.commands.FlyWheelOutCmd;


import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.WinchExtendCmd;
import frc.robot.commands.WinchRetractCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.HerderSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import java.time.Instant;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final FlyWheelSubsystem flyWheelSubsystem = new FlyWheelSubsystem();
  private final HerderSubsystem herderSubsystem = new HerderSubsystem();
  private final WinchSubsystem winchSubsystem = new WinchSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final SendableChooser<String> m_chooser;


  //Driver CONTROLLER ON PORT 0 AND Utilities ON PORT 1
  private final Joystick driverJoystickOne = new Joystick(OIConstants.kDriverControllerOnePort);
  private final Joystick driverJoystickTwo = new Joystick(OIConstants.kDriverControllerTwoPort);

  //chooser options
  private final String ampSideAuto = "Amp Side Auto";
  private final String centerAuto = "Center Auto";
  private final String feederSideAutoBlue = "Feeder Side Auto Blue";
  private final String feederSideAutoRed = "Feeder Side Auto Red";

  private final String leftAmpAuto = "Left Amp Auto";
  private final String getOutRightAuto = "Get Out Right";
 
  

  public RobotContainer() {
      m_chooser = new SendableChooser<>();
      m_chooser.addOption(ampSideAuto, ampSideAuto);
      m_chooser.addOption(centerAuto, centerAuto);
      m_chooser.addOption(feederSideAutoBlue, feederSideAutoBlue);
      m_chooser.addOption(feederSideAutoRed, feederSideAutoRed);

      m_chooser.addOption(leftAmpAuto, leftAmpAuto);
      m_chooser.addOption(getOutRightAuto, getOutRightAuto);

    
      //sets default command to joystick with feeders from controller
      swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
              swerveSubsystem,
              () -> -driverJoystickOne.getRawAxis(OIConstants.kDriverYAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kDriverXAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kDriverRotAxisXbox),
              () -> !driverJoystickOne.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

      armSubsystem.setDefaultCommand(new HoldArmCmd(armSubsystem));
      SmartDashboard.putData("Auto Choices", m_chooser);

      configureButtonBindings();
  }

  private void configureButtonBindings() {
    //////////Controller One\\\\\\\\\\\\\\\\
    //resets the gyro mid drive
    new JoystickButton(driverJoystickOne, OIConstants.kRestGyrobutton).whileTrue(new ResetGyroCmd(swerveSubsystem)); 
    //toggle on to pivot to the apriltag
    //make sure the april tag is IN VIEW
    new JoystickButton(driverJoystickOne, OIConstants.kShootSequenceButton).whileTrue(new LineUpCmd(swerveSubsystem, limelightSubsystem)); 

    //right and left trigger
    new JoystickButton(driverJoystickOne, OIConstants.kExtendLiftButton).whileTrue(new WinchExtendCmd(winchSubsystem));
    new JoystickButton(driverJoystickOne, OIConstants.kRetractLiftButton).whileTrue(new WinchRetractCmd(winchSubsystem));

    new JoystickButton(driverJoystickOne, OIConstants.kDriverResetArmButton).whileTrue(new InstantCommand(armSubsystem::resetArm));


    //////////Controller Two\\\\\\\\\\\\\\\\\
    //Y and A
    //this one runs herder and fly wheels at the same time
    new JoystickButton(driverJoystickTwo, OIConstants.kFlyWheelFwdButton).onTrue(new ShootCmd(flyWheelSubsystem,herderSubsystem));
    new JoystickButton(driverJoystickTwo, OIConstants.kFlyWheelBwdButton).whileTrue(new FlyWheelInCmd(flyWheelSubsystem));
    // if you want to just have a basic fly wheel shoot command use this one rather than the ShootCmd One
    // new JoystickButton(driverJoystickTwo, OIConstants.kFlyWheelFwdButton).whileTrue(new FlyWheelOutCmd(flyWheelSubsystem));

    //left and right trigger
    new JoystickButton(driverJoystickTwo, OIConstants.kArmForwardButton).whileTrue(new ArmFwdCmd(armSubsystem));
    new JoystickButton(driverJoystickTwo, OIConstants.kArmBackwardButton).whileTrue(new ArmBwdCmd(armSubsystem));
    //B and X
    new JoystickButton(driverJoystickTwo, OIConstants.kHerderInButton).whileTrue(new HerderInCmd(herderSubsystem));
    new JoystickButton(driverJoystickTwo, OIConstants.kHerderOutButton).whileTrue(new HerderOutCmd(herderSubsystem));
    

    //EACH OF THESE IS A PID SETPOINT    HOLD DOWN THE BUTTON TO GET TO THE RESPECTIVE POSITION
    //plus right
    new POVButton(driverJoystickTwo, OIConstants.kArmAmpButton).onTrue(new AngleAmpCmd(armSubsystem));
    //plus up
    new POVButton(driverJoystickTwo, OIConstants.kArmHerdButton).onTrue(new AngleHerdCmd(armSubsystem));
    //plus down
    new POVButton(driverJoystickTwo, OIConstants.kArmCloseSpeakerButton).onTrue(new AngleCloseSpeakerCmd(armSubsystem));
    //plus left
    new POVButton(driverJoystickTwo, OIConstants.kArmFarSpeakerButton).onTrue(new AngleFarSpeakerCmd(armSubsystem));

    //switches over to joystick using x button toggle
    // new JoystickButton(driverJoystickOne, 3).toggleOnTrue(new SwerveJoystickCmd(
    //           swerveSubsystem,
    //           () -> -driverJoystickTwo.getRawAxis(OIConstants.kDriverYAxis),
    //           () -> driverJoystickTwo.getRawAxis(OIConstants.kDriverXAxis),
    //           () -> driverJoystickTwo.getRawAxis(OIConstants.kDriverRotAxisJoystick),
    //           () -> !driverJoystickTwo.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); 


  }
  
  public Command getAutonomousCommand() {
    //1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(//
    AutoConstants.kMaxSpeedMetersPerSecond,//
    AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared)//
    .setKinematics(DriveConstants.kDriveKinematics);
    //2. Generate trajectory
    //maybe make more Autocommands that can pass in these values/cords
    
    //3. Define PID controllers for tracking trajectory
    //check out these autoconstants
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController,0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    HolonomicDriveController holoController = new HolonomicDriveController(xController, yController, thetaController);
    //4. Construct command to follow trajectory
  
    switch(m_chooser.getSelected()){
      ///////////////////////////////////////SIMPLE LEFT AUTO\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
      case "Amp Side Auto":
       Trajectory leftTrajectory = TrajectoryGenerator.generateTrajectory(//
        //x is forward/backward movement and y is left/right movement
        //coordinates are in meters i think???
        new Pose2d(0, 0, new Rotation2d(-45)),
        List.of(

        ),
        new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig);
        SwerveControllerCommand leftSwerveControllerCommand = new SwerveControllerCommand(//
    leftTrajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    holoController, 
    swerveSubsystem::setModuleStates, 
    swerveSubsystem);
    //5. Add some init qand wrap-up, and return everything
    return new SequentialCommandGroup(
      //resets odometer so that "even if the robot does not start on the initial point of our trajectory, it will move that trajectory to the current location"
      
      new InstantCommand(()-> swerveSubsystem.resetOdometry(leftTrajectory.getInitialPose())),
      //could use this instead
      // new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new AngleCloseSpeakerCmd(armSubsystem)),

      new AngleCloseSpeakerCmd(armSubsystem),
      //deadline finishes the group as soon as the first command finishes (in this case ShootCmd)
      new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new HoldArmCmd(armSubsystem)),
    ///
      // leftSwerveControllerCommand,//
      new InstantCommand(()-> swerveSubsystem.stopModules())//
    );
    ////////////////////////////////////////////SIMPLE CENTER AUTO\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    case "Feeder Side Auto Blue":
      Trajectory rightTrajectory = TrajectoryGenerator.generateTrajectory(//
        //x is forward/backward movement and y is left/right movement
        //coordinates are in meters i think???
        new Pose2d(0, 0, new Rotation2d(45)),
        List.of(
          new Translation2d(0,2)
        ),
        new Pose2d(3, 2, Rotation2d.fromDegrees(0)),
        trajectoryConfig);
        Trajectory rightFinishTrajectory = TrajectoryGenerator.generateTrajectory(//
        //x is forward/backward movement and y is left/right movement
        //coordinates are in meters i think???
        new Pose2d(-3, 3, new Rotation2d(0)),
        List.of(
          
        ),
        new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
        trajectoryConfig);
        SwerveControllerCommand rightSwerveControllerCommand = new SwerveControllerCommand(//
    rightTrajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    holoController, 
    swerveSubsystem::setModuleStates, 
    swerveSubsystem);
        SwerveControllerCommand rightFinsihSwerveControllerCommand = new SwerveControllerCommand(//
    rightFinishTrajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    holoController, 
    swerveSubsystem::setModuleStates, 
    swerveSubsystem);
    //5. Add some init qand wrap-up, and return everything
    return new SequentialCommandGroup(
      //resets odometer so that "even if the robot does not start on the initial point of our trajectory, it will move that trajectory to the current location"
      
      new InstantCommand(()-> swerveSubsystem.resetOdometry(rightTrajectory.getInitialPose())),
      //could use this instead
      // new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new AngleCloseSpeakerCmd(armSubsystem)),

      new AngleCloseSpeakerCmd(armSubsystem),
      //deadline finishes the group as soon as the first command finishes (in this case ShootCmd)
      new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new HoldArmCmd(armSubsystem)),
    ///
      new ParallelDeadlineGroup(rightSwerveControllerCommand
      // , new AngleHerdCmd(armSubsystem), new HerderInCmd(herderSubsystem)
      ),
      // new ParallelDeadlineGroup(rightFinsihSwerveControllerCommand, new AngleCloseSpeakerCmd(armSubsystem)),
      // new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new HoldArmCmd(armSubsystem)),


      //
      new InstantCommand(()-> swerveSubsystem.stopModules())//
    );
    case "Feeder Side Red":
      Trajectory rightRedTrajectory = TrajectoryGenerator.generateTrajectory(//
        //x is forward/backward movement and y is left/right movement
        //coordinates are in meters i think???
        new Pose2d(0, 0, new Rotation2d(-45)),
        List.of(
          new Translation2d(0,2)
        ),
        new Pose2d(3, 2, Rotation2d.fromDegrees(0)),
        trajectoryConfig);
        SwerveControllerCommand rightRedSwerveControllerCommand = new SwerveControllerCommand(//
    rightRedTrajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    holoController, 
    swerveSubsystem::setModuleStates, 
    swerveSubsystem);
    return new SequentialCommandGroup(
      //resets odometer so that "even if the robot does not start on the initial point of our trajectory, it will move that trajectory to the current location"
      
      new InstantCommand(()-> swerveSubsystem.resetOdometry(rightRedTrajectory.getInitialPose())),
      //could use this instead
      // new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new AngleCloseSpeakerCmd(armSubsystem)),

      new AngleCloseSpeakerCmd(armSubsystem),
      //deadline finishes the group as soon as the first command finishes (in this case ShootCmd)
      new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new HoldArmCmd(armSubsystem)),
    ///
      new ParallelDeadlineGroup(rightRedSwerveControllerCommand
      // , new AngleHerdCmd(armSubsystem), new HerderInCmd(herderSubsystem)
      ),
      new InstantCommand(()-> swerveSubsystem.stopModules())//
    );

    /////////////////////////////////////////////SIMPLE RIGHT AUTO\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    case "Center Auto":
      Trajectory centerTrajectory = TrajectoryGenerator.generateTrajectory(//
        //x is forward/backward movement and y is left/right movement
        //coordinates are in meters i think???
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          new Translation2d(0,3)
        ),
        new Pose2d(3, 3, Rotation2d.fromDegrees(0)),
        trajectoryConfig);
        Trajectory centerFinishTrajectory = TrajectoryGenerator.generateTrajectory(//
        //x is forward/backward movement and y is left/right movement
        //coordinates are in meters i think???
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          
        ),
        new Pose2d(-2, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig);
        SwerveControllerCommand centerSwerveControllerCommand = new SwerveControllerCommand(//
    centerTrajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    holoController, 
    swerveSubsystem::setModuleStates, 
    swerveSubsystem);
    SwerveControllerCommand centerFinishSwerveControllerCommand = new SwerveControllerCommand(//
    centerFinishTrajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    holoController, 
    swerveSubsystem::setModuleStates, 
    swerveSubsystem);
    //5. Add some init qand wrap-up, and return everything
    return new SequentialCommandGroup(
      //resets odometer so that "even if the robot does not start on the initial point of our trajectory, it will move that trajectory to the current location"
      
      new InstantCommand(()-> swerveSubsystem.resetOdometry(centerTrajectory.getInitialPose())),
      //could use this instead
      // new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new AngleCloseSpeakerCmd(armSubsystem)),

      new AngleCloseSpeakerCmd(armSubsystem),
      //deadline finishes the group as soon as the first command finishes (in this case ShootCmd)
      new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new HoldArmCmd(armSubsystem)),
      new ParallelDeadlineGroup(centerSwerveControllerCommand
      // , new HerderInCmd(herderSubsystem), new AngleHerdCmd(armSubsystem)
      ),
    ///
      // new ParallelDeadlineGroup(centerFinishSwerveControllerCommand, new AngleCloseSpeakerCmd(armSubsystem)),//
      // new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new AngleCloseSpeakerCmd(armSubsystem)),
      new InstantCommand(()-> swerveSubsystem.stopModules())//
    );
    case "Left Amp Auto":
      Trajectory leftLongTrajectory = TrajectoryGenerator.generateTrajectory(//
        //x is forward/backward movement and y is left/right movement
        //coordinates are in meters i think???
        new Pose2d(0, 0, new Rotation2d(-45)),
        List.of(
        ),
        new Pose2d(2.1336, -0.3, Rotation2d.fromDegrees(0)),
        trajectoryConfig);
        Trajectory ringOneToAmpTrajectory = TrajectoryGenerator.generateTrajectory(//
        //x is forward/backward movement and y is left/right movement
        //coordinates are in meters i think???
        new Pose2d(2.1336, -0.3, new Rotation2d(0)),
        List.of(
        ),
        new Pose2d(1.143, -0.9858, Rotation2d.fromDegrees(90)),
        trajectoryConfig);

        Trajectory ampGetOutTrajectory = TrajectoryGenerator.generateTrajectory(//
        //x is forward/backward movement and y is left/right movement
        //coordinates are in meters i think???
        new Pose2d(1.143, -0.9858, Rotation2d.fromDegrees(90)),
        List.of(
        ),
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig);

        SwerveControllerCommand leftLongSwerveControllerCommand = new SwerveControllerCommand(//
    leftLongTrajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    holoController, 
    swerveSubsystem::setModuleStates, 
    swerveSubsystem);
        SwerveControllerCommand ringOneToAmpSwerveControllerCommand = new SwerveControllerCommand(//
    ringOneToAmpTrajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    holoController, 
    swerveSubsystem::setModuleStates, 
    swerveSubsystem);
    SwerveControllerCommand ampGetOutSwerveControllerCommand = new SwerveControllerCommand(//
    ampGetOutTrajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    holoController, 
    swerveSubsystem::setModuleStates, 
    swerveSubsystem);
    //5. Add some init qand wrap-up, and return everything
    return new SequentialCommandGroup(
      //resets odometer so that "even if the robot does not start on the initial point of our trajectory, it will move that trajectory to the current location"
      
      new InstantCommand(()-> swerveSubsystem.resetOdometry(leftLongTrajectory.getInitialPose())),
      //could use this instead
      // new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new AngleCloseSpeakerCmd(armSubsystem)),

      new AngleCloseSpeakerCmd(armSubsystem),
      //deadline finishes the group as soon as the first command finishes (in this case ShootCmd)
      new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new HoldArmCmd(armSubsystem)),
    ///
      new ParallelDeadlineGroup(leftLongSwerveControllerCommand, new HerderInCmd(herderSubsystem),new AngleHerdCmd(armSubsystem)),//

      new ParallelDeadlineGroup(ringOneToAmpSwerveControllerCommand, new AngleAmpCmd(armSubsystem)),
      new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new AngleAmpCmd(armSubsystem)),
      new ParallelDeadlineGroup(ampGetOutSwerveControllerCommand, new AngleHerdCmd(armSubsystem)),

      new InstantCommand(()-> swerveSubsystem.stopModules())//
    );
    case"Get Out Right":
    Trajectory getOutTrajectory = TrajectoryGenerator.generateTrajectory(//
        //x is forward/backward movement and y is left/right movement
        //coordinates are in meters i think???
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            // new Translation2d(0,3)

        ),
        new Pose2d(0, 3, Rotation2d.fromDegrees(0)),
        trajectoryConfig);
        SwerveControllerCommand getOutSwerveControllerCommand = new SwerveControllerCommand(//
    getOutTrajectory, 
    swerveSubsystem::getPose, 
    DriveConstants.kDriveKinematics, 
    holoController, 
    swerveSubsystem::setModuleStates, 
    swerveSubsystem);
    //5. Add some init qand wrap-up, and return everything
    return new SequentialCommandGroup(
      //resets odometer so that "even if the robot does not start on the initial point of our trajectory, it will move that trajectory to the current location"
      
      new InstantCommand(()-> swerveSubsystem.resetOdometry(getOutTrajectory.getInitialPose())),
      //could use this instead
      // new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new AngleCloseSpeakerCmd(armSubsystem)),

      new AngleCloseSpeakerCmd(armSubsystem),
      //deadline finishes the group as soon as the first command finishes (in this case ShootCmd)
      new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new HoldArmCmd(armSubsystem)),
    ///
     getOutSwerveControllerCommand,//
      new InstantCommand(()-> swerveSubsystem.stopModules())//
    );

    default: return new SequentialCommandGroup(new AngleCloseSpeakerCmd(armSubsystem),new ParallelDeadlineGroup(new ShootCmd(flyWheelSubsystem, herderSubsystem), new HoldArmCmd(armSubsystem)));
    

      
   
  }
}
}