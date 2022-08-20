package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveVisualizer;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveVisualizer swerveVisualizer;

  SwerveDriveKinematics swerveDriveKinematics;

  Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  SwerveModuleState frontLeft = new SwerveModuleState();
  SwerveModuleState frontRight = new SwerveModuleState();
  SwerveModuleState backLeft = new SwerveModuleState();
  SwerveModuleState backRight = new SwerveModuleState();

  // var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, new
  // Rotation2d(turningEncoder.getDistance()));

  public SwerveSubsystem() {
    swerveDriveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation,
        backRightLocation);
    // TODO: Calculate Correct robot size based off of wheel locations. May want to
    // add a constructor to the Swerve Vis that accepts a SwerveDriveKinematics
    // object.

    swerveVisualizer = new SwerveVisualizer(2, 2);
  }

  public void swerveDrive(double xMetersPerSec, double yMetersPerSec, double rotationRadPerSec) {

    // Calculate what angle the wheels need to be and their speed based off our
    // desired robot velocities.
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xMetersPerSec, yMetersPerSec, rotationRadPerSec);
    SwerveModuleState[] moduleStates = swerveDriveKinematics.toSwerveModuleStates(desiredSpeeds);
    frontLeft = moduleStates[0];
    frontRight = moduleStates[1];
    backLeft = moduleStates[2];
    backRight = moduleStates[3];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: Replace Hardcoded position of robot with data from odometry.

    // Display the current desired location of all swerve modules.
    swerveVisualizer.update(frontLeft.angle, frontRight.angle, backLeft.angle, backRight.angle,
        new Pose2d(5, 5, new Rotation2d()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
