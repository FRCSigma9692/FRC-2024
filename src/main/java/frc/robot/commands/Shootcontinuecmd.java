package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class Shootcontinuecmd extends Command{
    public Shooter shooter;
    public Timer timer;
     public Intake intake;
     public double time;
     public double delay;
     public double shooter_pow;
     public double intake_pow;
    public Shootcontinuecmd(Shooter shooter,Intake intake, double time, double delay, double shooter_pow, double intake_pow){
        this.shooter = shooter;
        this.intake = intake;
        this.time = time;
        this.delay = delay;
        this.shooter_pow = shooter_pow;
        this.intake_pow = intake_pow;

    }
    
@Override
public void initialize(){
    timer = new Timer();
    timer.start();
    }

@Override
public void execute(){

    shooter.runShooter(shooter_pow, 0); //0.7
// if(timer.get()>delay) //0.75
//{ 
   intake.intake(0, intake_pow); //0.9
//}
    System.out.println(timer.get());


}

@Override
public void end(boolean Interrupted){

    shooter.runShooter(0, 0);
    intake.intake(0,0);
    timer.stop();
    timer.reset();
    System.out.println("over!");
   
}


@Override
public boolean isFinished(){
if(timer.get()> time) //1.7
return true;
else
return false;
}
}
