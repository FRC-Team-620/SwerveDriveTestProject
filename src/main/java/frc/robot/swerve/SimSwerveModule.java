package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.ISwerveModuleState;
import frc.robot.SwerveModuleHardwareSim;
import edu.wpi.first.math.MathUtil;

public class SimSwerveModule implements ISwerveModuleState {
    PIDController drivePID;
    ProfiledPIDController anglePID;

    SwerveModuleState desiredState;
    SwerveModuleHardwareSim sim;
    SwerveModuleAttributes attributes;
    String nTablesName = "";
    private static int ID_TRACKER = 0; 
    int id;
    public SimSwerveModule(SwerveModuleAttributes attributes){
        this.attributes = attributes;
        id = ID_TRACKER++;
        nTablesName = "SwerveModule" + id + "/";
        drivePID = new PIDController(Constants.driveKp, Constants.driveKi, Constants.driveKd);
        anglePID = new ProfiledPIDController(Constants.angleKp, Constants.angleKi, Constants.angleKd, new TrapezoidProfile.Constraints(100, 100));
        
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        drivePID.setIntegratorRange(-12, 12);
        anglePID.setIntegratorRange(-12, 12);
        sim = new SwerveModuleHardwareSim();
        
        SmartDashboard.putData(nTablesName + "Drive PID", drivePID);
        SmartDashboard.putData(nTablesName + "Caster PID", anglePID);
        setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0)));
        // SmartDashboard.putNumber("Turny Velocity", 12);
        // SmartDashboard.putNumber("Turny Acceleration", 12);
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState stateWithOffset = new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d( Math.PI / 180 * (state.angle.getDegrees() + attributes.angleOffset)));
        desiredState = SwerveModuleState.optimize(stateWithOffset, new Rotation2d(sim.getCasterAngleRad()));
        // desiredState = state;
        
        // Drive PID units are radians per second. 
        drivePID.setSetpoint(desiredState.speedMetersPerSecond/Constants.wheelRadius); 
        anglePID.setGoal(MathUtil.angleModulus(desiredState.angle.getRadians() - (Math.PI/180 * attributes.angleOffset)));
    }

    @Override
    public SwerveModuleState getActualState() {
        return new SwerveModuleState(sim.getWheelRadPerSec() * Constants.wheelRadius, new Rotation2d(MathUtil.angleModulus(sim.getCasterAngleRad())-(Math.PI/180*attributes.angleOffset)));
    }

    @Override
    public SwerveModuleState getDesiredState() {
        SwerveModuleState withoutOffset = new SwerveModuleState(desiredState.speedMetersPerSecond, new Rotation2d(Math.PI / 180 * (desiredState.angle.getDegrees()-attributes.angleOffset)));
        return withoutOffset;
    }

    @Override
    public void resetEncoders() {
        // TODO Auto-generated method stub
    }
    
    @Override
    public void update(){
        sim.update(0.02);
        double driveVolts = drivePID.calculate(sim.getWheelRadPerSec());
        double angleVolts = anglePID.calculate(MathUtil.angleModulus(sim.getCasterAngleRad() - (attributes.angleOffset * Math.PI/180)));
        
        sim.setWheelMotorVolts(driveVolts);
        sim.setCasterMotorVolts(angleVolts);
        // anglePID.setConstraints(new TrapezoidProfile.Constraints(SmartDashboard.getNumber("Turny Velocity", 12), SmartDashboard.getNumber("Turny Acceleration", 12)));
        SmartDashboard.putNumber(nTablesName + "Drive Volts", driveVolts);
        SmartDashboard.putNumber(nTablesName + "Angle Volts", angleVolts);
        SmartDashboard.putNumber(nTablesName + "Desired Speed", desiredState.speedMetersPerSecond/Constants.wheelRadius);
        SmartDashboard.putNumber(nTablesName + "Desired Angle", MathUtil.angleModulus(desiredState.angle.getRadians()));
        SmartDashboard.putNumber(nTablesName + "Angle", MathUtil.angleModulus(sim.getCasterAngleRad()));
        SmartDashboard.putNumber(nTablesName + "Speed", sim.getWheelRadPerSec());
        SmartDashboard.putNumber(nTablesName + "Angle Setpoint", anglePID.getSetpoint().position);
    }
}
