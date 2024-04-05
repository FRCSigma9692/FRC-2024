// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Subsystems.DriveSwerve;
import frc.robot.Subsystems.Hang;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Arm;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.ArmDown;
// import frc.robot.commands.ArmProfileCmd;
import frc.robot.commands.ArmShootCmd;
import frc.robot.commands.Drivecmd;
import frc.robot.commands.IntakeCmd;
// import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SensorIntakeCmd;
import frc.robot.commands.Sensorbtnintake;
import frc.robot.commands.Shootcontinuecmd;
import frc.robot.commands.ShooterCmd;

public class RobotContainer {
  public boolean flag = false;
  // public DriveSubsystemPathplanner robot =  new DriveSubsystemPathplanner();
  
  public DriveSwerve robot = new DriveSwerve();
  public Shooter shooter = new Shooter();
  public Intake intake = new Intake(); 
  public Arm mech = new Arm();
  public Hang hanger = new Hang();
  public ShooterCmd Shoot = new ShooterCmd(shooter, intake, 1.7, 0.9, 0.75,0.9); 
  public Shootcontinuecmd ShootContinue = new Shootcontinuecmd(shooter, intake, 14, 0.9, 0.3,0.9); 
 public SensorIntakeCmd intake_and_sense = new SensorIntakeCmd(intake, 0.8, intake.sensor , 4 );

  // public SensorIntakeCmd intake2 = new SensorIntakeCmd(intake, 0.8, intake.sensor , 10 );
  public Drivecmd drivecmd = new Drivecmd(robot);
   public Sensorbtnintake intake3 = new Sensorbtnintake(intake, 0.9, intake.sensor );


  public XboxController m_driverController = new XboxController(0);
 public XboxController m_driverController2 = new XboxController(1);
 
//  public DistanceSensor distSensor = new DistanceSensor();

public double offset;

 private SendableChooser<Command> autoChooser;

  public RobotContainer() {

    NamedCommands.registerCommand("shoot", Shoot);
    NamedCommands.registerCommand("arm", new ArmCmd(mech, 98));
    NamedCommands.registerCommand("Q59_Shoot", new ShooterCmd(shooter, intake, 2.5, 0, 0.22, 0.7));
    NamedCommands.registerCommand("Shootcontinue", ShootContinue);
    NamedCommands.registerCommand("armB1", new ArmCmd(mech, 104));
    NamedCommands.registerCommand("arm1temp", new ArmShootCmd(mech, 90));
    NamedCommands.registerCommand("arm1tempshoot", new ParallelCommandGroup(new ArmShootCmd(mech, 90), new ShooterCmd(shooter,  intake, 2, 1.5, 0.75, 0.9)));

    NamedCommands.registerCommand("IntakeContinue",new IntakeCmd(intake, 1, 0.9));
    NamedCommands.registerCommand("armFarB1", new ArmCmd(mech, 105));

    NamedCommands.registerCommand("Arm1up", new ShooterCmd(shooter, intake, 1.7, 0.9, 0.65,0.9));
    NamedCommands.registerCommand("armB_one", new ArmCmd(mech, 98));

    NamedCommands.registerCommand("ArmforMiddle", new ArmCmd(mech, 98));

    
    NamedCommands.registerCommand("armDown", new ArmDown(mech, 67));

    NamedCommands.registerCommand("armforAuto2", new ArmCmd(mech, 91.5));
    NamedCommands.registerCommand("armFar", new ArmCmd(mech, 106));
    NamedCommands.registerCommand("Brakemod",drivecmd);

    NamedCommands.registerCommand("armFarB3", new ArmCmd(mech, 108.5));
    NamedCommands.registerCommand("IntakeSense", new SensorIntakeCmd(intake, 0.7, intake.sensor, 4.5));
    autoChooser = AutoBuilder.buildAutoChooser("Blue1");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
     
        robot.setDefaultCommand(
        
        new RunCommand(
            () -> robot.drive(
                (Math.abs(m_driverController.getLeftY()) > 0.06? -m_driverController.getLeftY(): 0),
                (Math.abs(m_driverController.getLeftX()) > 0.06? -m_driverController.getLeftX(): 0),
                (Math.abs(m_driverController.getRightX()) > 0.06? -m_driverController.getRightX(): 0),
                true,false),
            robot));
       
  }

  private void configureBindings() {

  //Driver 1
  new JoystickButton(m_driverController, Button.kCircle.value)
  .whileTrue(new RunCommand(() -> robot.gyro.reset(), robot));
  
  new JoystickButton(m_driverController, Button.kTriangle.value)
  .whileTrue(new RunCommand(() -> shooter.runShooter(0.7,0), shooter));
  
  new Trigger(() -> m_driverController.getRightTriggerAxis()>0.7)
  .whileTrue(new RunCommand(() -> shooter.runShooter(0.6,0), shooter)); 
  
  new JoystickButton(m_driverController, Button.kR1.value)
  .whileTrue(new RunCommand(() -> robot.Xshape(), robot));

  // Driver 2

  new JoystickButton(m_driverController2, Button.kR1.value)
  .onTrue((intake3));

  new JoystickButton(m_driverController2, Button.kCross.value)
  .onTrue(new RunCommand(() -> mech.armTo(168), mech)); //0.55
  
  new JoystickButton(m_driverController2, Button.kL1.value)
  .onTrue(new RunCommand(()-> mech.armDown(0.6), mech));//new RunCommand(() -> mech.armTo(91.5), mech)); //0.55
  
  new JoystickButton(m_driverController2, Button.kSquare.value)
  .onTrue(new RunCommand(() -> mech.armTo(91.5), mech)); //0.55

  new JoystickButton(m_driverController2, Button.kCircle.value)
  .whileTrue(new RunCommand(() -> mech.ll2SetArm(), mech));
   
  new JoystickButton(m_driverController2, Button.kTriangle.value)
  .whileTrue(new ParallelCommandGroup(new RunCommand(() -> shooter.runShooter(0.3,0), shooter),
  (new RunCommand(() -> intake.intake(0,0.5), intake)))); //Right is Intake , left is outtake
   
    //POV

    new POVButton(m_driverController2, 0)
    .whileTrue(new RunCommand(()-> mech.upwithabsenc(0.6), mech));
    
    new POVButton(m_driverController2, 180)
    .whileTrue(new RunCommand(()-> mech.downwithabsenc(0.6), mech));

    new POVButton(m_driverController2, 90)
    .whileTrue(new RunCommand(() -> shooter.runShooter(0, 0.1), shooter));

    new POVButton(m_driverController2, 270)
    .onTrue(new RunCommand(() -> mech.armTo(120), mech));

    //Triggers
    new Trigger(() -> m_driverController2.getLeftTriggerAxis()>0.05)
    .whileTrue(new RunCommand(() -> intake.intake(0,m_driverController2.getLeftTriggerAxis()), intake));
 
    new Trigger(() -> m_driverController2.getRightTriggerAxis()>0.05)
    .whileTrue(new RunCommand(() -> intake.intake(m_driverController2.getRightTriggerAxis(),0), intake));

    new Trigger(() -> (Math.abs(m_driverController2.getLeftY())>0.05 || Math.abs(m_driverController2.getRightY())>0.05))
    .whileTrue(new RunCommand(() -> hanger.set(-m_driverController2.getLeftY(),-m_driverController2.getRightY()), hanger));
    
  }
   public Command getAutonomousCommand() {
     robot.resetPose(new Pose2d(1.35,5.5,new Rotation2d(0)));
      
        // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new SequentialCommandGroup(autoChooser.getSelected());
  
}
}