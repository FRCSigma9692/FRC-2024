package frc.robot.Sensors;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensor extends SubsystemBase{
    Rev2mDistanceSensor sensor;
    public DistanceSensor(){
        sensor = new Rev2mDistanceSensor(Port.kOnboard);
        sensor.setAutomaticMode(true);
        sensor.setEnabled(true);
    }


    public void init(){
        
    }

    public double getDistance(){
        double dist = sensor.getRange();
        return dist;

    }
}
