package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends SubsystemBase{
    CANSparkMax intake1;
    public RelativeEncoder Enc1;
    CANSparkMax intake2;
    public RelativeEncoder Enc2;
    boolean flag;
    public DigitalInput sensor;
    // public CANSparkMax LED = new CANSparkMax(19, MotorType.kBrushed);
    //  public ColourMatch colourMatch1 = new ColourMatch();
    public Intake(){
        intake1 = new CANSparkMax(16, MotorType.kBrushless);
        Enc1 = intake1.getEncoder();

        intake2 = new CANSparkMax(15, MotorType.kBrushless);
        Enc2 = intake2.getEncoder();

        sensor= new DigitalInput(0);
    }

    public void intake(double B1, double B2){
        intake1.set((B1 -B2));
        intake2.set((B1 -B2));
    }

    public void runOne(double a){
        intake2.set(-a);
    }

    public void changeColour(boolean x){
      
      if(x)
flag = !flag;

    // if( colourMatch1.red> 0.45 && colourMatch1.blue < 0.2){
    //        SmartDashboard.putString("MOTORS", "STOP");
    //        intake(0,0);
            }

    public boolean isOn1(){
      return (intake1.get() == 0);
    }

    public boolean isOn2(){
      return (intake2.get() == 0);
    }
  
    public void sensorintake(){

    }
    public void DigitalDisplay(){
      SmartDashboard.putBoolean("DigitalSensor", sensor.get());
      // if(!sensor.get()){
      //   LED.set(0.1);
      // }
      // else{
      //   LED.set(0);
      // }
    }
}

