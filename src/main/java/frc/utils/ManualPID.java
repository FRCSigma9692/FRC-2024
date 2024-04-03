    package frc.utils;
public class ManualPID {

    private final double maxAcceleration = 0.1; // Maximum acceleration rate (adjust as needed)
    private final double maxVelocity = 1.0; // Maximum velocity (adjust as needed)
    private double currentVelocity = 0.0;
    private double setpoint = 0.0; // Store setpoint
    private double currentPosition = 0.0; // Store current position

    public ManualPID(){
    }

    public void gradualAcceleration() {
        // Calculate acceleration based on the stored setpoint and current position
        double acceleration = calculateAcceleration();

        // Apply acceleration to current velocity
        currentVelocity += acceleration;

        // Limit the velocity to the maximum velocity
        currentVelocity = Math.min(currentVelocity, maxVelocity);

        // Control arm movement based on the current velocity
        controlArm(currentVelocity);

        // Optional: Add a delay if needed to control the rate of acceleration
        // sleep(milliseconds);
    }

    public void controlArm(double pow){

    }

    public double calculateAcceleration() {
        // Calculate the error between the setpoint and current position
        double error = setpoint - currentPosition;
    
        // Define your proportional gain
        double kP = 0.1; // Adjust as needed
    
        // Calculate acceleration using proportional control
        double acceleration = kP * error;
    
        // Limit acceleration to maximum value
        acceleration = Math.min(acceleration, maxAcceleration);
    
        // Return the calculated acceleration
        return acceleration;
    }
    public void Deceleration(){
        
    }

    // Setter methods for setpoint and currentPosition if needed
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setCurrentPosition(double currentPosition) {
        this.currentPosition = currentPosition;
    }
}
