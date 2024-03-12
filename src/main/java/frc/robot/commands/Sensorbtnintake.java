package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
public class Sensorbtnintake extends Command {

        Intake intake;
        double spd;
        DigitalInput sensor;
        boolean flag ;
        Timer timer;
        public Sensorbtnintake(Intake intake, double speed, DigitalInput sensor){
            this.intake = intake;
            spd = speed;
            this.sensor = sensor;
            flag = false;
            addRequirements(intake);
        }
    
        @Override
        public void initialize(){
        }
    
        @Override
        public void execute(){
            if(sensor.get()){
            intake.intake(0, spd);}
            // System.out.println("Sensor-based intake running");
            //  System.out.println(timer.get());
        }
    
        @Override
        public void end(boolean interrupted){
            intake.intake(0, 0);
            flag = !flag;
            System.out.println("Sensor-based intake stopped");
        }
    
        @Override
        public boolean isFinished(){
            if(!sensor.get())
            return true;
            else
            return false;
        }
           
    }
