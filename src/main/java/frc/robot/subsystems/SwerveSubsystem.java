package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveVisualizer;
import frc.robot.swerve.SimSwerveModule;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveVisualizer swerveVisualizer;

  SwerveDriveKinematics swerveDriveKinematics;

  SwerveDriveOdometry swerveDriveOdometry;
  Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  SimSwerveModule[] modules = new SimSwerveModule[]{new SimSwerveModule(Constants.frontLeftAttributes), new SimSwerveModule(Constants.frontRightAttributes), new SimSwerveModule(Constants.backLeftAttributes), new SimSwerveModule(Constants.backRightAttributes)};
  
  boolean fieldCentered;
  
  public SwerveSubsystem(boolean mode) {
    swerveDriveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation,
        backRightLocation);
      fieldCentered = mode;
    // TODO: Calculate Correct robot size based off of wheel locations. May want to
    // add a constructor to the Swerve Vis that accepts a SwerveDriveKinematics
    // object.	
    swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, new Rotation2d(0), new Pose2d(5.0, 5.0, new Rotation2d()));
    swerveVisualizer = new SwerveVisualizer(2, 2);
    
  }

  public void swerveDrive(double xMetersPerSec, double yMetersPerSec, double rotationRadPerSec) {
  	// Calculate what angle the wheels need to be and their speed based off our
    // desired robot velocities.
    ChassisSpeeds desiredSpeeds;
    
    // Calculate the desired chassis speeds from the inputs, be they field centered or not. 
    if(fieldCentered){
      desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSec, yMetersPerSec, rotationRadPerSec, swerveDriveOdometry.getPoseMeters().getRotation());
    } 
    else{
      desiredSpeeds = new ChassisSpeeds(xMetersPerSec, yMetersPerSec, rotationRadPerSec);
    }

    // If the commanded robot speed is 0, leave the modules facing the direction they are facing but put the speeds to 0. 
    if (desiredSpeeds.vxMetersPerSecond == 0 && desiredSpeeds.vyMetersPerSecond == 0 && desiredSpeeds.omegaRadiansPerSecond == 0) {
      brake();
    }
    else{
      // calculate the chassis speeds needed from the chassis speeds, and set them. 
      SwerveModuleState[] desiredModuleStates = swerveDriveKinematics.toSwerveModuleStates(desiredSpeeds);
      normalizeDrive(desiredModuleStates, desiredSpeeds);
      setModuleStates(desiredModuleStates);
    }
    

    // Update the swerve odometry based on the module states. 
    swerveDriveOdometry.update(
      //TODO: when implementing on hardware, use gyroscope value, not commanded turn rate. 
      swerveDriveOdometry.getPoseMeters().getRotation().plus(new Rotation2d(rotationRadPerSec*0.02)), 
      getModuleStates());
  }

  public void normalizeDrive(SwerveModuleState[] desiredStates, ChassisSpeeds speeds) {
    double translationalK = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / Constants.maxTraverseVelocityMetersPerSecond;
    double rotationalK = Math.abs(speeds.omegaRadiansPerSecond) / Constants.maxRotationRadsPerSecond;
    double k = Math.max(translationalK, rotationalK);

    // Find the how fast the fastest spinning drive motor is spinning                                       
    double realMaxSpeed = 0.0;
    for (SwerveModuleState moduleState : desiredStates) {
      realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
    }

    double scale = Math.min(k * Constants.maxWheelVelocityMetersPerSecond / realMaxSpeed, 1);
    for (SwerveModuleState moduleState : desiredStates) {
      moduleState.speedMetersPerSecond *= scale;
    }
  }

  public void brake() {
    SwerveModuleState[] modulestates = getModuleStates();
    double realMaxSpeed = 0.0;
    for (SwerveModuleState moduleState : modulestates) {
      realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
    }
    if (realMaxSpeed > Constants.xModeMaxSpeed){
      setModuleStates(new SwerveModuleState[]{new SwerveModuleState(0, modules[0].getActualState().angle), new SwerveModuleState(0, modules[1].getActualState().angle), new SwerveModuleState(0, modules[2].getActualState().angle), new SwerveModuleState(0, modules[3].getActualState().angle)});
    }
    else{
      SwerveModuleState[] states = new SwerveModuleState[4];
      Double[] angles = new Double[]{45.0, 135.0, 135.0, 45.0};
      for (int i = 0; i<=3; i++){
        if (Constants.squareMode)
          states[i] = new SwerveModuleState(0, new Rotation2d(Math.PI/180 * (angles[i] + 90)));
        else
          states[i] = new SwerveModuleState(0, new Rotation2d(Math.PI/180 * angles[i]));
      }
      setModuleStates(states);
    }
    
  }
  public void tankDriveSet(double leftSpeed, double rightSpeed) {
    setModuleStates(new SwerveModuleState[]{new SwerveModuleState(leftSpeed, new Rotation2d(0)), new SwerveModuleState(rightSpeed, new Rotation2d(0)), new SwerveModuleState(rightSpeed, new Rotation2d(0)), new SwerveModuleState(leftSpeed, new Rotation2d(0))});
    // drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void setFieldOriented(boolean mode){
    fieldCentered = mode;
  }

  public boolean getFieldOriented(){
    return fieldCentered;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.maxWheelVelocityMetersPerSecond);

      for (int i = 0; i <= 3; i++) {
        modules[i].setDesiredState(desiredStates[i]);
      }
  }
  
  public SwerveModuleState[] getDesiredStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i <= 3; i++) {
      states[i++] = modules[i].getDesiredState();
    }
    return states;
  }

  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i <= 3; i++) {
      states[i] = modules[i].getActualState();
    }

    return states;
  }

  @Override
  public void periodic() {

    for (int i = 0; i <= 3; i++) {
      modules[i].update();
    }
    swerveVisualizer.update(modules[0].getActualState().angle, modules[1].getActualState().angle, modules[2].getActualState().angle, modules[3].getActualState().angle, swerveDriveOdometry.getPoseMeters());//new Pose2d(5, 5, new Rotation2d())
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
