package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;

public class ArmDown extends Command{
    Arm arm;
    double sp;
    public ArmDown(Arm arm, double setPoint){
        this.arm = arm;
        sp = setPoint;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        SmartDashboard.getString("status", "Executed");
        arm.armTo(sp);
        // arm.UpGetPos();`````````````1
        SmartDashboard.putString("Arm", "NOT Over!");
    }

    @Override
    public void end(boolean interrupted){
        arm.armUp(0);
        SmartDashboard.putString("Arm", "Over");
    }

    @Override
    public boolean isFinished(){
        if(Math.toDegrees(arm.l_UpAbsoluteEncoder.getPosition()) <= sp-2 || Math.toDegrees(arm.l_UpAbsoluteEncoder.getPosition()) <= sp+2 ) 
        // if(arm.l_Up.get() ==0 && arm.r_Up.get() == 0)
        return true;
        else
        return false;
    }
    
}
