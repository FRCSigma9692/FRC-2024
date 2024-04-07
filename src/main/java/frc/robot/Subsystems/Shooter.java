package frc.robot.Subsystems;


import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
   public CANSparkMax r_pulley;
   public RelativeEncoder r_enc;
   SparkPIDController r_pid;
   public CANSparkMax l_pulley;
   public RelativeEncoder l_enc;
   SparkPIDController l_pid;
    double timer;

    public  Shooter(){
        
        r_pulley = new CANSparkMax(13, MotorType.kBrushless);
        r_enc = r_pulley.getEncoder();
        r_pid = r_pulley.getPIDController();
        r_pid.setFeedbackDevice(r_enc);
        r_pid.setP(1);
        r_pid.setFF(1);
        r_pulley.burnFlash();
        // r_pulley_pid.setPositionPIDWrappingEnabled(true);

      l_pulley = new CANSparkMax(12, MotorType.kBrushless);
      l_enc = l_pulley.getEncoder();
      l_pid = l_pulley.getPIDController();
      l_pid.setFeedbackDevice(l_enc);
       l_pid.setP(1);
       l_pid.setFF(1);
       l_pulley.burnFlash();
        // l_pulley_pid.setPositionPIDWrappingEnabled(true); 
    }

    public void runShooter(double B2, double B1){
        l_pulley.set(B1 - B2);
        r_pulley.set(B1 - B2);

}
 
public void runShooterAuto(double B1){
      // l_pid.setReference(B1, ControlType.kVelocity);
      // r_pid.setReference(B1, ControlType.kVelocity);

}

public void runshooterDPAD(int POV){
    if(POV == 270){
        SmartDashboard.putString("Shoot", "out");
        runShooter(0.6, 0);
    }
    else if(POV == 90){
        SmartDashboard.putString("Shoot", "in");
        runShooter(0, 0.6);
    }
    else{
        SmartDashboard.putString("Shoot", "false");
        runShooter(0, 0);
    }
}

public void displaySpeed(){
        SmartDashboard.putNumber("rShooterSpeed", r_enc.getVelocity()) ;
        SmartDashboard.putNumber("lShooterSpeed", l_enc.getVelocity()) ;
}
}