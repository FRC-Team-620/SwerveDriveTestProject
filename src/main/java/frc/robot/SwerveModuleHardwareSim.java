package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleHardwareSim {

    FlywheelSim casterSim, wheelSim;
    private double casterAngleRad = 0;
    private double wheelRadPerSec = 0;
    private double casterMotorVolts, wheelMotorVolts;

    public static final double kDriveGearRatio = 8.16;
    public static final double kTurnGearRatio = 12.8;



    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double kvVoltSecondsPerRadian = 0.16;
    public static final double kaVoltSecondsSquaredPerRadian = 0.0348;


    //Drive

    public static final double DrivekvVoltSecondsPerMeter = 2.3;
    public static final double DrivekaVoltSecondsSquaredPerMeter = 0.0917;
    // Hacked together simulator so students are able to test and learn about pid
    // loops for swerve.
    public SwerveModuleHardwareSim() {
        casterSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian),DCMotor.getNEO(1),kTurnGearRatio);
        wheelSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(DrivekvVoltSecondsPerMeter,DrivekaVoltSecondsSquaredPerMeter),DCMotor.getNEO(1),kDriveGearRatio);
        
    }

    public void setCasterMotorVolts(double volts) {
        casterMotorVolts = MathUtil.clamp(volts, -12, 12);
    }

    public void setWheelMotorrVolts(double volts) {
        wheelMotorVolts = MathUtil.clamp(volts, -12, 12);
    }

    public double getWheelRadPerSec() {
        return wheelRadPerSec/kDriveGearRatio;
    }

    public double getCasterAngleRad() {
        return casterAngleRad/kTurnGearRatio;
    }

    public void update(double dt) {
        wheelSim.setInputVoltage(wheelMotorVolts);
        casterSim.setInputVoltage(casterMotorVolts);

        wheelSim.update(dt);
        casterSim.update(dt);

        casterAngleRad += wheelSim.getAngularVelocityRadPerSec() * dt;
        wheelRadPerSec = wheelSim.getAngularVelocityRadPerSec();
    }
}