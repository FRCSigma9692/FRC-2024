package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;


public class IntakeCmd extends Command{
     Intake intake;
    double sp;
    Timer timer;
    double spd;
    public IntakeCmd(Intake intake, double time, double speed){
        this.intake = intake;
        sp = time;
        spd = speed;
    }

    @Override
    public void initialize(){
        timer = new Timer();
        timer.start();

    }

    @Override
    public void execute(){
        intake.intake(0, spd);
    }

    @Override
    public void end(boolean interrupted){
        timer.stop();
        timer.reset();
        intake.intake(0, 0);
    }

    @Override
    public boolean isFinished(){
        if(timer.get() >= sp)
        return true;
        else
        return false;
    }
       
}
