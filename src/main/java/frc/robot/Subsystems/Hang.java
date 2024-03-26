package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase{
    public CANSparkMax HangL;
    public CANSparkMax HangR;
    RelativeEncoder Enc_R;
    RelativeEncoder Enc_L;
    public Hang(){
        HangR = new CANSparkMax(17 , MotorType.kBrushless);
        Enc_R = HangR.getEncoder();
        HangL = new CANSparkMax(18 , MotorType.kBrushless);
        Enc_L = HangL.getEncoder();

    }


    void HangTo(double sp){
        if((Math.abs(Enc_R.getPosition()) + Math.abs(Enc_L.getPosition()))/2  > sp+5){
            HangR.set(0.75);
            HangL.set(-0.75);
        }
        else if((Math.abs(Enc_R.getPosition()) + Math.abs(Enc_L.getPosition()))/2  < sp-5){
            HangR.set(-0.5);
            HangL.set(0.5);}
        else{
            HangR.set(0);
            HangL.set(0);
        }
    }

    public void set(double B1){
            HangR.set(-B1);
            HangL.set(B1);
    }
    public void set(double JL, double JR){
       if(JL > 0.1 && ((Enc_L.getPosition())< 320)){
    
        HangL.set(0.95);
       }
       else if(JL < -0.1 && ((Enc_L.getPosition())>3)){
        
        HangL.set(-0.95);
       }
       else{
        HangL.set(0);
       }

       if(JR > 0.1 && ((Enc_R.getPosition())>-320)){
        HangR.set(-0.95);
       }
       else if(JR < -0.1 && ((Enc_R.getPosition())<-3)){
        HangR.set(0.95);
       }
       else{
        HangR.set(0);
       }
    }

    public void display(){
        SmartDashboard.putNumber("HangL", Enc_L.getPosition());
        SmartDashboard.putNumber("HangR", Enc_R.getPosition());

    }
}
