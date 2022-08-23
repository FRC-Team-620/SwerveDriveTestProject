package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveVisualizer;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveVisualizer swerveVisualizer;

  SwerveDriveKinematics swerveDriveKinematics;

  SwerveDriveOdometry swerveDriveOdometry;

  /** Creates a new ExampleSubsystem. */
  Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  //ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);
  

  SwerveModuleState frontLeft = new SwerveModuleState();
  SwerveModuleState frontRight = new SwerveModuleState();
  SwerveModuleState backLeft = new SwerveModuleState();
  SwerveModuleState backRight = new SwerveModuleState();
  
  //var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, new Rotation2d(turningEncoder.getDistance()));

  public SwerveSubsystem() {
    swerveDriveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, new Rotation2d(0), new Pose2d(5.0, 5.0, new Rotation2d()));
    swerveVisualizer = new SwerveVisualizer(2, 2);
  }

  public void swerveDrive(double x, double y, double theta) {
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(x, y, theta);
    SwerveModuleState[] moduleStates = swerveDriveKinematics.toSwerveModuleStates(desiredSpeeds);
    frontLeft = moduleStates[0];
    frontRight = moduleStates[1];
    backLeft = moduleStates[2];
    backRight = moduleStates[3];
    swerveDriveOdometry.update(swerveDriveOdometry.getPoseMeters().getRotation().plus(new Rotation2d(theta)), moduleStates);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(frontLeft.angle);
    System.out.println(frontRight.angle);
    System.out.println(backLeft.angle);
    System.out.println(backRight.angle);
    swerveVisualizer.update(frontLeft.angle, frontRight.angle, backLeft.angle, backRight.angle, swerveDriveOdometry.getPoseMeters());//new Pose2d(5, 5, new Rotation2d())
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
