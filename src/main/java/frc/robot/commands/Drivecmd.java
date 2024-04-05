package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSwerve;

public class Drivecmd extends Command {

    DriveSwerve drive;
    public  Drivecmd( DriveSwerve drive){

        this.drive = drive;
    }
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        drive.Xshape();
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }


}
