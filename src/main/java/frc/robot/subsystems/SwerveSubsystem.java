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

  SimSwerveModule[] modules = new SimSwerveModule[] {
      new SimSwerveModule(Constants.frontLeftAttributes),
      new SimSwerveModule(Constants.frontRightAttributes),
      new SimSwerveModule(Constants.backLeftAttributes),
      new SimSwerveModule(Constants.backRightAttributes)
  };

  boolean fieldCentered;

  /**
   * The constructor for SwerveSubsystem.
   * 
   * @param mode The starting FieldCentered state of the robot. 
   */
  public SwerveSubsystem(boolean mode) {
    swerveDriveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation,
        backRightLocation);
    fieldCentered = mode;
    // TODO: Calculate Correct robot size based off of wheel locations. May want to
    // add a constructor to the Swerve Vis that accepts a SwerveDriveKinematics
    // object.
    swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, new Rotation2d(0),
        new Pose2d(5.0, 5.0, new Rotation2d()));
    swerveVisualizer = new SwerveVisualizer(2, 2);

  }

  /**
   * The SwerveDrive method determines which drive type to use, or whether to
   * brake, and updates the odometry.
   * 
   * @param xMetersPerSec     The commanded speed to move in the X direction in
   *                          meters per second.
   * @param yMetersPerSec     The commanded speed to move in the Y direction in
   *                          meters per second.
   * @param rotationRadPerSec The commanded rotation speed of the robot in radians
   *                          per second.
   */

  public void swerveDrive(double xMetersPerSec, double yMetersPerSec, double rotationRadPerSec) {
    // Calculate what angle the wheels need to be and their speed based off our
    // desired robot velocities.
    ChassisSpeeds desiredSpeeds;

    // Calculate the desired chassis speeds from the inputs, be they field centered
    // or not.
    if (fieldCentered) {
      desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSec, yMetersPerSec, rotationRadPerSec,
          swerveDriveOdometry.getPoseMeters().getRotation());
    } else {
      desiredSpeeds = new ChassisSpeeds(xMetersPerSec, yMetersPerSec, rotationRadPerSec);
    }

    // If the commanded robot speed is 0, leave the modules facing the direction
    // they are facing but put the speeds to 0.
    if (desiredSpeeds.vxMetersPerSecond == 0 && desiredSpeeds.vyMetersPerSecond == 0
        && desiredSpeeds.omegaRadiansPerSecond == 0) {
      brake();
    } else {
      // calculate the chassis speeds needed from the chassis speeds, and set them.
      SwerveModuleState[] desiredModuleStates = swerveDriveKinematics.toSwerveModuleStates(desiredSpeeds);
      normalizeDrive(desiredModuleStates, desiredSpeeds);
      setModuleStates(desiredModuleStates);
    }

    // Update the swerve odometry based on the module states.
    swerveDriveOdometry.update(
        // TODO: when implementing on hardware, use gyroscope value, not commanded turn
        // rate.
        swerveDriveOdometry.getPoseMeters().getRotation().plus(new Rotation2d(rotationRadPerSec * 0.02)),
        getModuleStates());
  }

  /**
   * This normalizes the the speeds to not be commanding the modules to spin
   * faster than they should be.
   * 
   * @param desiredStates The desired states of the robot, will be scaled so the
   *                      fastest one is moving at the max possible wheel move
   *                      speed.
   * @param speeds        The commanded chassis speeds of the robot.
   */
  public void normalizeDrive(SwerveModuleState[] desiredStates, ChassisSpeeds speeds) {
    double translationalK = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        / Constants.maxTraverseVelocityMetersPerSecond;
    double rotationalK = Math.abs(speeds.omegaRadiansPerSecond) / Constants.maxRotationRadsPerSecond;
    double k = Math.max(translationalK, rotationalK);

    // Find the how fast the fastest spinning drive motor is being commanded to
    // spin.
    double realMaxSpeed = 0.0;
    for (SwerveModuleState moduleState : desiredStates) {
      realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
    }

    double scale = Math.min(k * Constants.maxWheelVelocityMetersPerSecond / realMaxSpeed, 1);
    for (SwerveModuleState moduleState : desiredStates) {
      moduleState.speedMetersPerSecond *= scale;
    }
  }

  /**
   * Brake mode keeps the wheels in the same orientations, but uses the drive PID
   * loops to counteract the motion of the robot.
   * This method sets the desired module states to the braking states, but returns
   * immediately, and does not wait for the wheels to reach that position.
   * Once the robot speed is below the constant xModeMaxSpeed, the robot's wheels
   * will go into an X shape.
   */
  public void brake() {
    SwerveModuleState[] modulestates = getModuleStates();
    double realMaxSpeed = 0.0;
    for (SwerveModuleState moduleState : modulestates) {
      realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
    }
    if (realMaxSpeed > Constants.xModeMaxSpeed) {
      setModuleStates(new SwerveModuleState[] { new SwerveModuleState(0, modules[0].getActualState().angle),
          new SwerveModuleState(0, modules[1].getActualState().angle),
          new SwerveModuleState(0, modules[2].getActualState().angle),
          new SwerveModuleState(0, modules[3].getActualState().angle) });
    } else {
      SwerveModuleState[] states = new SwerveModuleState[4];
      Double[] angles = new Double[] { 45.0, 135.0, 135.0, 45.0 };
      for (int i = 0; i <= 3; i++) {
        if (Constants.squareMode)
          states[i] = new SwerveModuleState(0, new Rotation2d(Math.PI / 180 * (angles[i] + 90)));
        else
          states[i] = new SwerveModuleState(0, new Rotation2d(Math.PI / 180 * angles[i]));
      }
      setModuleStates(states);
    }

  }

  /**
   * Please don't use this. It would maybe work. Maybe.
   * It hypothetically locks all the modules to forwards and runs both sides at
   * the set speeds.
   * This version uses a lot of wheel slip so it's a bad idea.
   * 
   * @param leftSpeed  The speed to move the modules on the right side of the
   *                   robot.
   * @param rightSpeed The speed to move the modules on the right side of the
   *                   robot.
   */
  public void tankDriveSet(double leftSpeed, double rightSpeed) {
    setModuleStates(
        new SwerveModuleState[] {
            new SwerveModuleState(leftSpeed, new Rotation2d(0)),
            new SwerveModuleState(rightSpeed, new Rotation2d(0)),
            new SwerveModuleState(rightSpeed, new Rotation2d(0)),
            new SwerveModuleState(leftSpeed, new Rotation2d(0)),
        });
    // drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * This is slightly less bad than the tankDriveSet method. Its basically just
   * robot orriented drive but with only one axis. I still hate writing this.
   * 
   * @param speedMetersPerSecond The speed to move the robot forwards and
   *                             backwards.
   * @param turnRadsPerSecond    The speed to turn the robot.
   */
  public void curvatureDriveSet(double speedMetersPerSecond, double turnRadsPerSecond) {
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(speedMetersPerSecond, 0, turnRadsPerSecond);
    if (desiredSpeeds.vxMetersPerSecond == 0 && desiredSpeeds.omegaRadiansPerSecond == 0) {
      brake();
    } else {
      // calculate the chassis speeds needed from the chassis speeds, and set them.
      SwerveModuleState[] desiredModuleStates = swerveDriveKinematics.toSwerveModuleStates(desiredSpeeds);
      normalizeDrive(desiredModuleStates, desiredSpeeds);
      setModuleStates(desiredModuleStates);
    }
  }

  /**
   * Sets the fieldCentered boolean to determine whether the robot is moving
   * field-centered or not.
   * 
   * @param mode
   */
  public void setFieldOriented(boolean mode) {
    fieldCentered = mode;
  }

  /**
   * Gets the current fieldCentered state of the robot.
   * 
   * @return Whether or not the robot is in field orriented mode.
   */
  public boolean getFieldOriented() {
    return fieldCentered;
  }

  /**
   * This sets the desired module states to the desired module states, after
   * desaturating them.
   * 
   * @param desiredStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.maxWheelVelocityMetersPerSecond);

    for (int i = 0; i <= 3; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  /**
   * This gets the current desired states of the robot.
   * 
   * @return the current desired states of the robot.
   */
  public SwerveModuleState[] getDesiredStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i <= 3; i++) {
      states[i++] = modules[i].getDesiredState();
    }
    return states;
  }

  /**
   * This gets the current actual states of the modules.
   * 
   * @return The module states.
   */
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
    swerveVisualizer.update(modules[0].getActualState().angle, modules[1].getActualState().angle,
        modules[2].getActualState().angle, modules[3].getActualState().angle, swerveDriveOdometry.getPoseMeters());// new
                                                                                                                   // Pose2d(5,
                                                                                                                   // 5,
                                                                                                                   // new
                                                                                                                   // Rotation2d())
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
