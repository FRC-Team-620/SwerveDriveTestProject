package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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


  SwerveModuleState frontLeftState = new SwerveModuleState();
  SwerveModuleState frontRightState = new SwerveModuleState();
  SwerveModuleState backLeftState = new SwerveModuleState();
  SwerveModuleState backRightState = new SwerveModuleState();

  SimSwerveModule frontLeft = new SimSwerveModule(Constants.frontLeftCAN);
  SimSwerveModule frontRight = new SimSwerveModule(Constants.frontRightCAN);
  SimSwerveModule backLeft = new SimSwerveModule(Constants.backLeftCAN);
  SimSwerveModule backRight = new SimSwerveModule(Constants.backRightCAN);
  
  // var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, new
  // Rotation2d(turningEncoder.getDistance()));
  
  public SwerveSubsystem() {
    swerveDriveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation,
        backRightLocation);
    // TODO: Calculate Correct robot size based off of wheel locations. May want to
    // add a constructor to the Swerve Vis that accepts a SwerveDriveKinematics
    // object.	
    swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, new Rotation2d(0), new Pose2d(5.0, 5.0, new Rotation2d()));
    swerveVisualizer = new SwerveVisualizer(2, 2);
  }

  public void swerveDrive(double xMetersPerSec, double yMetersPerSec, double rotationRadPerSec) {
  	// Calculate what angle the wheels need to be and their speed based off our
    // desired robot velocities.
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xMetersPerSec, yMetersPerSec, rotationRadPerSec);
    SwerveModuleState[] actualModuleStates = new SwerveModuleState[]{frontLeft.getActualState(), frontRight.getActualState(), backLeft.getActualState(), backRight.getActualState()};
    SwerveModuleState[] commandedModuleStates = swerveDriveKinematics.toSwerveModuleStates(desiredSpeeds);
    frontLeftState = commandedModuleStates[0];
    frontRightState = commandedModuleStates[1];
    backLeftState = commandedModuleStates[2];
    backRightState = commandedModuleStates[3];
	swerveDriveOdometry.update(swerveDriveOdometry.getPoseMeters().getRotation().plus(new Rotation2d(rotationRadPerSec*0.02)), actualModuleStates);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontLeft.setDesiredState(frontLeftState);
    frontRight.setDesiredState(frontRightState);
    backLeft.setDesiredState(backLeftState);
    backRight.setDesiredState(backRightState);

    frontLeft.update();
    frontRight.update();
    backLeft.update();
    backRight.update();
    swerveVisualizer.update(frontLeft.getActualState().angle, frontRight.getActualState().angle, backLeft.getActualState().angle, backRight.getActualState().angle, swerveDriveOdometry.getPoseMeters());//new Pose2d(5, 5, new Rotation2d())
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
