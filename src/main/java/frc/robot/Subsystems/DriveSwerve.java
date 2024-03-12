package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;

public class DriveSwerve extends SubsystemBase{
    private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;


     public final MaxSwerveModule fl = new MaxSwerveModule(1,2,Math.toRadians(-90), false); //Offset in radians  0
     public final MaxSwerveModule fr = new MaxSwerveModule(3,4,Math.toRadians(0), false); // 90
     public final MaxSwerveModule bl = new MaxSwerveModule(7,8,Math.toRadians(180), false); // 270
     public final MaxSwerveModule br = new MaxSwerveModule(5,6,Math.toRadians(90), false);  //180
     public AHRS gyro = new AHRS();
     public MaxSwerveModule[] states= new MaxSwerveModule[4];
     SwerveDriveOdometry odo = new SwerveDriveOdometry(
     DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(yaw()),
    new SwerveModulePosition[]{ 
     fl.getPosition(), 
     fr.getPosition(), 
     bl.getPosition(),
     br.getPosition()});

     public DriveSwerve(){
          gyro.reset();
         setStates();
      gyro.reset();
       AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
           this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {

              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
      
     }

StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();


     public void setStates(){
      states[0] = fl;
      states[1] = fr;
      states[2] = bl;
      states[3] = br;
      }

     @Override
     public void periodic(){
        odo.update(Rotation2d.fromDegrees(yaw()),
        new SwerveModulePosition[]{
            fl.getPosition(),
            fr.getPosition(),
            bl.getPosition(),
            br.getPosition()});
            SmartDashboard.putNumber("Xmoved", getPose().getTranslation().getX());
            SmartDashboard.putNumber("Ymoved", getPose().getTranslation().getY());
            publisher.set(getModuleStates());
     }

    public void resetPose(Pose2d pose){
        odo.resetPosition(Rotation2d.fromDegrees(yaw()), new SwerveModulePosition[]{
            fl.getPosition(),
            fr.getPosition(),
            bl.getPosition(),
            br.getPosition()
        }, pose);
     }

     public void drive(double xspd, double yspd, double rot, boolean fieldcentric, boolean rateLimit){
          double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(yspd, xspd);
      double inputTranslationMag = Math.sqrt(Math.pow(xspd, 2) + Math.pow(yspd, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xspd;
      ySpeedCommanded = yspd;
      m_currentRotation = rot;
    }



        xspd = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        rot = m_currentRotation* DriveConstants.kMaxAngularSpeed;
        yspd = ySpeedCommanded* DriveConstants.kMaxSpeedMetersPerSecond;
        var States = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldcentric
        ?ChassisSpeeds.fromFieldRelativeSpeeds(xspd, yspd,rot, Rotation2d.fromDegrees(yaw()))
        :new ChassisSpeeds(xspd, yspd, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
        States, DriveConstants.kMaxSpeedMetersPerSecond); 
        fl.setDesiredState(States[0]);
        fr.setDesiredState(States[1]);
        bl.setDesiredState(States[2]);
        br.setDesiredState(States[3]);
        
        SmartDashboard.putNumber("fl_angle", States[0].angle.getDegrees());
        SmartDashboard.putNumber("fr_angle", States[1].angle.getDegrees());
        SmartDashboard.putNumber("bl_angle", States[2].angle.getDegrees());
        SmartDashboard.putNumber("br_angle", States[3].angle.getDegrees());

        SmartDashboard.putNumber("Rot", rot);
     }

     public void setSwerveStates(SwerveModuleState[] states){
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
      fl.setDesiredState(states[0]);
      fr.setDesiredState(states[1]);
      bl.setDesiredState(states[2]);
      br.setDesiredState(states[3]);
     }

     public Pose2d getPose(){
       return odo.getPoseMeters();
     }

     public ChassisSpeeds getRobotRelativeSpeeds(){
      return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
     }

     public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
      ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
      SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
      setSwerveStates(targetStates);
}

public SwerveModuleState[] getModuleStates() {
  SwerveModuleState[] states11 = new SwerveModuleState[states.length];
  for (int i = 0; i < states.length; i++) {
    states11[i] = states[i].getState();
  }
  return states11;
}


     public double yaw(){
      return (-gyro.getYaw());
     }

     public void Xshape(){
      fl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      fr.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      bl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      br.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }
}

