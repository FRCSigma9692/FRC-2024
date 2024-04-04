package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;

public class ArmProfileCmd extends Command{
    Arm arm;
    double sp;
    double curpos;
    double error;
    double accelDist;
    double Accelpow;
    double decelDist;
    double decelRate;
    double decelpow=0.02;
    double stablepow;
    double Attainedspeed;
    double pos;

    double dist;
    public ArmProfileCmd(Arm arm, double setPoint,double accelDist, double decelDist){
        this.arm = arm;
        sp = setPoint;
        dist = accelDist;
        this.decelDist = decelDist;
    }

    @Override
    public void initialize(){
         Accelpow = 0.02;
        curpos = Math.toDegrees(arm.l_UpAbsoluteEncoder.getPosition());
        pos = curpos;
        error  = sp-curpos;
        accelDist = curpos + (error*dist);
        decelDist = sp - (error*decelDist);
        stablepow = 0.6;
    }

    @Override
    public void execute(){

        

        //stage 3
        if(error>0){
        System.out.println(error+">0");
            if(curpos <= accelDist){

                if(Accelpow<0.6){   
                Accelpow = 0.035 * (curpos - (pos-3));
                Attainedspeed = arm.l_Up.get();
                SmartDashboard.putString("Phase","Acceleration" );                
                }
               else if(Accelpow>=0.6){
                Accelpow = 0.6;
               }
            arm.l_Up.set(Accelpow);
            }
            else if(curpos >= accelDist && curpos<=decelDist){

                    arm.l_Up.set(Attainedspeed);
                SmartDashboard.putString("Phase","stable" );                

            }
            else if(curpos >= decelDist && curpos <sp){
                decelpow = Attainedspeed/(sp - curpos);
                if(decelpow>0.002){
                double pow =Attainedspeed - decelpow;
                arm.l_Up.set(pow);
                }else if(decelpow<0.002){
                arm.l_Up.set(0);
                }
                SmartDashboard.putString("Phase","deceleration" );                

            }
    }
        else if(error<0){
            // System.out.println(error+"<0");

            if(curpos >= accelDist){
            
            if(-Accelpow>-0.6){
            Accelpow = 0.035 * (pos+3 - curpos);
                
                SmartDashboard.putString("Phase","AccelerationDown" );
            }
            else if(-Accelpow<-0.6){
                Accelpow = 0.6;
            }
            arm.l_Up.set(-Accelpow);
            Attainedspeed = arm.l_Up.get();
            }
            else if(curpos <= accelDist && curpos>= decelDist){
            arm.l_Up.set(Attainedspeed);
            SmartDashboard.putString("Phase","stableDown" ); 
        }
        else if(curpos<=decelDist && curpos>sp){
                decelpow = Math.abs(Attainedspeed/(curpos - sp));
                if(decelpow>0.002){
                double pow =Attainedspeed + decelpow;
                arm.l_Up.set(pow);
                }else if(decelpow<0.002){
                arm.l_Up.set(0);
                }
                SmartDashboard.putString("Phase","decelerationDown" ); 
        }
    }
        
        // arm.UpGetPos();`````````````1
        SmartDashboard.putNumber("SpeedAttained", Attainedspeed);
        curpos = Math.toDegrees(arm.l_UpAbsoluteEncoder.getPosition());
        System.out.println(error);
      //  error  = sp-curpos;
    }

    @Override
    public void end(boolean interrupted){
        arm.upwithabsenc(0);
        SmartDashboard.putString("Arm", "Over");
        System.out.println("over");
        SmartDashboard.putNumber("ArmValue", curpos);
    }

    @Override
    public boolean isFinished(){
        if(Math.toDegrees(arm.l_UpAbsoluteEncoder.getPosition()) < sp+1 && Math.toDegrees(arm.l_UpAbsoluteEncoder.getPosition()) > sp-1)
        return true;
        else
        return false;
    }
    
}
