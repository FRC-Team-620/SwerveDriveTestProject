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

public class SimSwerveModule implements ISwerveModuleState {
    PIDController drivePID;
    ProfiledPIDController anglePID;

    SwerveModuleState desiredState;
    SwerveModuleHardwareSim sim;
    public SimSwerveModule(){
        drivePID = new PIDController(0.1, 0, 0);
        anglePID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(6.0, 6.0));
        anglePID.enableContinuousInput(0, 2*Math.PI);
        sim = new SwerveModuleHardwareSim();
        
        SmartDashboard.putData(drivePID);
        SmartDashboard.putData(anglePID);
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        // TODO Auto-generated method stub
        desiredState = state;
        TrapezoidProfile.State goal = new TrapezoidProfile.State(desiredState.angle.getRadians(), 0);
        drivePID.setSetpoint(desiredState.speedMetersPerSecond);
        anglePID.setGoal(goal);
    }

    @Override
    public SwerveModuleState getModuleState() {
        // TODO Auto-generated method stub
        return new SwerveModuleState(sim.getWheelRadPerSec()* Constants.wheelRadius, new Rotation2d(sim.getCasterAngleRad()));
    }

    @Override
    public SwerveModuleState getDesiredState() {
        // TODO Auto-generated method stub
        return desiredState;
    }

    @Override
    public void resetEncoders() {
        // TODO Auto-generated method stub
    }
    
    @Override
    public void update(){
        sim.update(0.02);
        
        sim.setWheelMotorrVolts(drivePID.calculate(sim.getWheelRadPerSec()));
        sim.setCasterMotorVolts(anglePID.calculate(sim.getCasterAngleRad()%2*Math.PI));

        SmartDashboard.putNumber("Desired Speed", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Desired Angle", desiredState.angle.getRadians());
    }
}
