// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.TimedRobot;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Sensors.ColourMatch;
//import frc.robot.Subsystems.DriveSubsystemPathplanner;
//import frc.robot.Subsystems.DriveSwerve;
//import frc.robot.Thread

public class Robot extends TimedRobot {
 boolean flag = false;
  double Angle;
  double Distance;
  double leftHeight;
  double rightHeight;
  double ThresholdDist = 10;
  double kp = -0.01;

  ColourMatch color = new ColourMatch();



  //private DriveSwerve robot = new DriveSwerve();
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {


    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    
    SmartDashboard.putNumber("lefty", m_robotContainer.m_driverController.getLeftY());
    SmartDashboard.putNumber("leftx", m_robotContainer.m_driverController.getLeftX());
 SmartDashboard.putNumber("rightx", m_robotContainer.m_driverController.getRightX());
  //  m_robotContainer.mech.UpGetPos();
   m_robotContainer.mech.UpGetPosABS();
   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //  NetworkTable table2 = NetworkTableInstance.getDefault().getTable("CAM");
   NetworkTableEntry tx = table.getEntry("tx");
   NetworkTableEntry ty = table.getEntry("ty");
   NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
   double x = tx.getDouble(0.0);
   double y = ty.getDouble(0.0);
   double area = ta.getDouble(0.0);

SmartDashboard.putNumber("ArmOffset", m_robotContainer.offset);

//post to smart dashboard periodically
  SmartDashboard.putNumber("LimelightX", x);
  SmartDashboard.putNumber("LimelightY", y);
  SmartDashboard.putNumber("LimelightArea", area);
  SmartDashboard.putNumber("gyro", m_robotContainer.robot.yaw());
CommandScheduler.getInstance().run();

m_robotContainer.hanger.display();
m_robotContainer.intake.DigitalDisplay();



}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      //double timer = Timer.getFPGATimestamp();
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);



//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.mech.setDefaultCommand(
      new RunCommand(() -> m_robotContainer.mech.armUp(0) , m_robotContainer.mech));
    
      m_robotContainer.shooter.setDefaultCommand(
      new RunCommand(() -> m_robotContainer.shooter.runShooter(0,0),m_robotContainer.shooter));


    m_robotContainer.intake.setDefaultCommand( 
      new RunCommand(() -> m_robotContainer.intake.intake(0,0), m_robotContainer.intake));
  
      m_robotContainer.hanger.setDefaultCommand( 
      new RunCommand(() -> m_robotContainer.hanger.set(0), m_robotContainer.hanger));
  }


  @Override
  public void teleopPeriodic() {
      if (m_robotContainer.m_driverController.getLeftBumper()){
        DriveConstants.kMaxSpeedMetersPerSecond = DriveConstants.kSlowSpeedMetersPersecond;
      }
      else{
        DriveConstants.kMaxSpeedMetersPerSecond = DriveConstants.kFastSpeedMetersPerSecond;
      }

      //  if (m_robotContainer.m_driverController.getLeftTriggerAxis()>0.1){
      //   DriveConstants.kMaxSpeedMetersPerSecond =DriveConstants.kFastSpeedMetersPerSecond - DriveConstants.kFastSpeedMetersPerSecond * m_robotContainer.m_driverController.getLeftTriggerAxis() ;
      // }
      // else{
      //   DriveConstants.kMaxSpeedMetersPerSecond = DriveConstants.kFastSpeedMetersPerSecond;
      // }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit(){}

  @Override
  public void simulationPeriodic() {
  
  }

}
/* FOR AUTO-LIMELIGHT
Angle = Math.toRadians(0 + y);
Distance = (Constants.MechanismConstatns.TgtHt - Constants.MechanismConstatns.CamHt) / Math.tan(Angle);
SmartDashboard.putNumber("Tgt_Dist", Distance);
swerve_d.setModulePositions(90, 90, 90, 90);
if(leftHeight>(rightHeight+2)){
    swerve_d.setPow(0.3);
    Angle = Math.toRadians(0 + y);
    Distance = (Constants.MechanismConstatns.TgtHt - Constants.MechanismConstatns.CamHt) / Math.tan(Angle);
    double error = ThresholdDist - Distance;
    swerve_d.setModulePositions(90 + kp*error, 90- kp*error, 90 - kp*error, 90 + kp*error);// kp is -ve

  }
  else if((leftHeight+2)<rightHeight){
    swerve_d.setPow(-0.3);
    Angle = Math.toRadians(0 + y);
    Distance = (Constants.MechanismConstatns.TgtHt - Constants.MechanismConstatns.CamHt) / Math.tan(Angle);
    double error = ThresholdDist - Distance;
    swerve_d.setModulePositions(90 + kp*error, 90 - kp*error, 90 - kp*error, 90 + kp*error);// kp is -ve
  
  }
  else{
    swerve_d.setPow(0);
  }*/
