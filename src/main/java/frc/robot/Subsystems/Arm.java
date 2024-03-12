package frc.robot.Subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
double i = 1;
    public CANSparkMax l_Up;
    public SparkPIDController l_Up_pid;
    // public RelativeEncoder l_Up_enc;
    public AbsoluteEncoder l_UpAbsoluteEncoder;

    public CANSparkMax r_Up;
    public SparkPIDController r_Up_pid;
    // public RelativeEncoder r_Up_enc;
    public AbsoluteEncoder r_UpAbsoluteEncoder;


//initial height starts from 65
    public Arm(){


    //     //for relative encoder 
    //    r_Up = new CANSparkMax(9, MotorType.kBrushless);
    //     r_Up_enc = r_Up.getEncoder();
    //     r_Up_pid = r_Up.getPIDController();
    //     r_Up_pid.setFeedbackDevice(r_Up_enc);
    //     r_Up.setIdleMode(IdleMode.kBrake);
    //     // r_Up_pid.setP(2);
    //     // r_Up_pid.setFF(2);
    //     r_Up_pid.setPositionPIDWrappingEnabled(true);
    //     r_Up.burnFlash();

    //     l_Up = new CANSparkMax(10, MotorType.kBrushless);
    //     l_Up_enc = l_Up.getEncoder();
    //     l_Up_pid = l_Up.getPIDController();
    //     l_Up_pid.setFeedbackDevice(l_Up_enc);
    //     l_Up.setIdleMode(IdleMode.kBrake);
    //     // l_Up_pid.setP(2);
    //     // l_Up_pid.setFF(2);
    //     l_Up_pid.setPositionPIDWrappingEnabled(true);
    //     l_Up.burnFlash();

        //for absolute encodder
         l_Up = new CANSparkMax(9, MotorType.kBrushless);
        l_UpAbsoluteEncoder = l_Up.getAbsoluteEncoder(Type.kDutyCycle);
        l_Up_pid = l_Up.getPIDController();
        l_Up_pid.setFeedbackDevice(l_UpAbsoluteEncoder);
        l_Up.setIdleMode(IdleMode.kBrake);
        l_Up_pid.setP(1);
        l_Up_pid.setFF(0);
        l_Up_pid.setPositionPIDWrappingEnabled(true);
        l_Up.burnFlash();

        r_Up = new CANSparkMax(10, MotorType.kBrushless);
        r_UpAbsoluteEncoder = r_Up.getAbsoluteEncoder(Type.kDutyCycle);
        r_Up_pid = r_Up.getPIDController();
        r_Up_pid.setFeedbackDevice(r_UpAbsoluteEncoder);
        r_Up.setIdleMode(IdleMode.kBrake);
        r_Up_pid.setP(1);
        r_Up_pid.setFF(0);
        r_Up_pid.setPositionPIDWrappingEnabled(true);
        r_Up.burnFlash();

       

        
    }

    public void armUp(double B1){
        double pos = (((Math.toDegrees(l_UpAbsoluteEncoder.getPosition()) + Math.toDegrees(r_UpAbsoluteEncoder.getPosition())))/2.0);
        if(pos<180){
            l_Up.set((B1));  //-(b1-0)   
            r_Up.set((B1));} //(b1-0)
        else{
            l_Up.set(0);
            r_Up.set(0);
        }

    }

        public void armDown(double B1){
        double pos = (((Math.toDegrees(l_UpAbsoluteEncoder.getPosition()) + Math.toDegrees(r_UpAbsoluteEncoder.getPosition())))/2.0);
        if(pos>65){
            l_Up.set(-(B1));  //-(b1-0)   
            r_Up.set(-(B1));} //(b1-0)
        else{
            l_Up.set(0);
            r_Up.set(0);
        }
    }

//    public void armSource(){
//         if((((Math.abs(l_UpAbsoluteEncoder.getPosition()) + Math.abs(r_UpAbsoluteEncoder.getPosition())))/2.0)>123){
//             l_Up.set((0.5));
//             r_Up.set(-(0.5));
//         }
//         else if((((Math.abs(l_UpAbsoluteEncoder.getPosition()) + Math.abs(r_UpAbsoluteEncoder.getPosition())))/2.0)<120){
//             l_Up.set(-(0.9));
//             r_Up.set((0.9));
//         }
//         else{
//              l_Up.set(0);
//             r_Up.set(0);
//             SmartDashboard.putString("SRC","SRC");
//         }
//     }

    // public void armDown(){
    //     if((((Math.abs(l_UpAbsoluteEncoder.getPosition()) + Math.abs(r_UpAbsoluteEncoder.getPosition())))/2.0)> 6){
    //         l_Up.set((0.75));
    //         r_Up.set(-(0.75));
    //     }
    //     else{
    //         l_Up.set(0);
    //         r_Up.set(0);
    //         SmartDashboard.putString("DOWN","DOWN");
    //     }
    //}

    public void armTo(double sp){// Amp is 4.211 for left n 4.248 for right 243 deg
        double pos = (((Math.toDegrees(l_UpAbsoluteEncoder.getPosition()) + Math.toDegrees(r_UpAbsoluteEncoder.getPosition())))/2.0);
        if(pos> sp+10){
            l_Up.set(-0.7);
            r_Up.set(-0.7);
        }
        else if(pos< sp-10){
            l_Up.set(0.7);
            r_Up.set(0.7);


        }
        else{
            l_Up.set(0);
            r_Up.set(0);
            SmartDashboard.putString("DOWN","DOWN");
        }
    }
   
    

    public void display(double js){
        SmartDashboard.putNumber("JS", js);
    }
    
    public void UpGetPosABS(){
        SmartDashboard.putNumber("l_Up", Math.toDegrees(l_UpAbsoluteEncoder.getPosition()));   //31.499735
        SmartDashboard.putNumber("r_Up", Math.toDegrees(r_UpAbsoluteEncoder.getPosition())); //-31.071171
        SmartDashboard.putNumber("l_up - r-up",Math.toDegrees(l_UpAbsoluteEncoder.getPosition())-Math.toDegrees(r_UpAbsoluteEncoder.getPosition()));

    }

    public void upwithabsenc(double speed){
    double pos = (Math.toDegrees(l_UpAbsoluteEncoder.getPosition()) + Math.toDegrees(r_UpAbsoluteEncoder.getPosition()))/2;

    if(pos <180){
        l_Up.set(speed);
        r_Up.set(speed);
    }
    else{
        l_Up.set(0);
        r_Up.set(0);
        }
    }
    public void downwithabsenc(double speed){
    double pos = (Math.toDegrees(l_UpAbsoluteEncoder.getPosition()) + Math.toDegrees(r_UpAbsoluteEncoder.getPosition()))/2;
    if(pos >65){

        l_Up.set(-speed);
        r_Up.set(-speed);
    }
    else{
        l_Up.set(0);
        r_Up.set(0);
        }
    }
}