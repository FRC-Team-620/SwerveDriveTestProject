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

  // SwerveModuleState[] desiredModuleStates = new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
  // SwerveModuleState frontLeftState = new SwerveModuleState();
  // SwerveModuleState frontRightState = new SwerveModuleState();
  // SwerveModuleState backLeftState = new SwerveModuleState();
  // SwerveModuleState backRightState = new SwerveModuleState();

  SimSwerveModule[] modules = new SimSwerveModule[]{new SimSwerveModule(), new SimSwerveModule(), new SimSwerveModule(), new SimSwerveModule()};
  // SimSwerveModule frontLeft = new SimSwerveModule();
  // SimSwerveModule frontRight = new SimSwerveModule();
  // SimSwerveModule backLeft = new SimSwerveModule();
  // SimSwerveModule backRight = new SimSwerveModule();
  boolean fieldCentered;
  // var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, new
  // Rotation2d(turningEncoder.getDistance()));
  
  public SwerveSubsystem(boolean mode) {
    swerveDriveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation,
        backRightLocation);
      fieldCentered = mode;
    // TODO: Calculate Correct robot size based off of wheel locations. May want to
    // add a constructor to the Swerve Vis that accepts a SwerveDriveKinematics
    // object.	
    swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, new Rotation2d(0), new Pose2d(5.0, 5.0, new Rotation2d()));
    swerveVisualizer = new SwerveVisualizer(2, 2);
    SmartDashboard.putBoolean("brake", false);
    
  }

  public void swerveDrive(double xMetersPerSec, double yMetersPerSec, double rotationRadPerSec) {
  	// Calculate what angle the wheels need to be and their speed based off our
    // desired robot velocities.
    ChassisSpeeds desiredSpeeds;

    System.out.println("" + xMetersPerSec + yMetersPerSec + rotationRadPerSec);

    if(fieldCentered){
      desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSec, yMetersPerSec, rotationRadPerSec, swerveDriveOdometry.getPoseMeters().getRotation());
    } 
    else{
      desiredSpeeds = new ChassisSpeeds(xMetersPerSec, yMetersPerSec, rotationRadPerSec);
    }

    // System.out.println(swerveDriveOdometry.getPoseMeters().getRotation().plus(new Rotation2d(rotationRadPerSec*0.02)));
    // System.out.println(getModuleStates());

    


    if (desiredSpeeds.vxMetersPerSecond == 0 && desiredSpeeds.vyMetersPerSecond == 0 && desiredSpeeds.omegaRadiansPerSecond == 0) {
      SmartDashboard.putBoolean("/brake", true);
      brake();
      return;
    }
    SmartDashboard.putBoolean("/brake", false);
    SwerveModuleState[] desiredModuleStates = swerveDriveKinematics.toSwerveModuleStates(desiredSpeeds);
    // normalizeDrive(desiredModuleStates, desiredSpeeds);
    setModuleStates(desiredModuleStates);
    swerveDriveOdometry.update(
      swerveDriveOdometry.getPoseMeters().getRotation().plus(new Rotation2d(rotationRadPerSec*0.02)), 
      getModuleStates());
  }

  public void normalizeDrive(SwerveModuleState[] desiredStates, ChassisSpeeds speeds) {
    double translationalK = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / Constants.maxVelocityMetersPerSecond;
    double rotationalK = Math.abs(speeds.omegaRadiansPerSecond) / Constants.maxRotationRadsPerSecond;
    double k = Math.max(translationalK, rotationalK);

    // Find the how fast the fastest spinning drive motor is spinning                                       
    double realMaxSpeed = 0.0;
    for (SwerveModuleState moduleState : desiredStates) {
      realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
    }

    double scale = Math.min(k * Constants.maxVelocityMetersPerSecond / realMaxSpeed, 1);
    for (SwerveModuleState moduleState : desiredStates) {
      moduleState.speedMetersPerSecond *= scale;
    }
  }

  public void brake() {
    System.out.println(new SwerveModuleState(0, modules[0].getActualState().angle));
    setModuleStates(new SwerveModuleState[]{new SwerveModuleState(0, modules[0].getActualState().angle), new SwerveModuleState(0, modules[1].getActualState().angle), new SwerveModuleState(0, modules[2].getActualState().angle), new SwerveModuleState(0, modules[3].getActualState().angle)});
    // for (SimSwerveModule module : modules) {
    //   module.setDesiredState(new SwerveModuleState(0, module.getActualState().angle));
    // }
  }

  public void setFieldOriented(boolean mode){
    fieldCentered = mode;
  }

  public boolean getFieldOriented(){
    return fieldCentered;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // SwerveDriveKinematics.desaturateWheelSpeeds(
    //     desiredStates, Constants.maxVelocityMetersPerSecond);

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
    // This method will be called once per scheduler run
    // SwerveModuleState[] desiredModuleStates = getDesiredStates();
    // System.out.println(desiredModuleStates);
    // SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, Constants.maxVelocityMetersPerSecond);
    // setModuleStates(desiredModuleStates);

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
