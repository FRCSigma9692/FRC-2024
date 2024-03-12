package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MaxSwerveModule{
    public CANSparkMax driveMotor;
    public CANSparkMax turnMotor;

    private RelativeEncoder driveEncoder;
    private AbsoluteEncoder turnEncoder;
    
    private SparkPIDController turnPID;
    private SparkPIDController drivePID;
    private double angularOffset = 0.0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    public boolean inverted;

    public MaxSwerveModule(int drivingID, int turnID, double offset, boolean inverted){
        driveMotor = new CANSparkMax(drivingID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        
        drivePID = driveMotor.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);
        drivePID.setP(0.04);

        turnPID = turnMotor.getPIDController();
        turnPID.setFeedbackDevice(turnEncoder);
        turnPID.setP(0.3);

        driveMotor.burnFlash();
        turnMotor.burnFlash();

        angularOffset = offset;
        desiredState.angle = new Rotation2d(turnEncoder.getPosition());
        driveEncoder.setPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition() - angularOffset));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition() , new Rotation2d(turnEncoder.getPosition() - angularOffset));
    }

    public void setDesiredState(SwerveModuleState state){

        SwerveModuleState correctedstate = new SwerveModuleState();
        correctedstate.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedstate.angle = state.angle.plus(Rotation2d.fromRadians(angularOffset));

        SwerveModuleState optimized = SwerveModuleState.optimize(correctedstate, new Rotation2d(turnEncoder.getPosition()));      
        turnPID.setReference(optimized.angle.getRadians(), ControlType.kPosition);
        drivePID.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity);
SmartDashboard.putNumber("Speed", correctedstate.speedMetersPerSecond);
    }

    public void resetEnc(){
        driveEncoder.setPosition(0);
    }
}
