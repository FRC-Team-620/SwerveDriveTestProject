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
    int canID;
    public SimSwerveModule(int CANID){
        drivePID = new PIDController(Constants.driveKp, Constants.driveKi, Constants.driveKd);
        anglePID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(6.0, 6.0));
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        sim = new SwerveModuleHardwareSim();
        canID = CANID;
        SmartDashboard.putData(""+CANID, drivePID);
        SmartDashboard.putData(""+CANID, anglePID);
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        // TODO Auto-generated method stub
        desiredState = state;
        TrapezoidProfile.State goal = new TrapezoidProfile.State(MathUtil.angleModulus(desiredState.angle.getRadians()), 0);
        drivePID.setSetpoint(desiredState.speedMetersPerSecond/Constants.wheelRadius);
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
        
        sim.setWheelMotorVolts(drivePID.calculate(sim.getWheelRadPerSec()));
        sim.setCasterMotorVolts(anglePID.calculate(sim.getCasterAngleRad()%2*Math.PI));

        SmartDashboard.putNumber("Desired Speed" + canID, desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Desired Angle" + canID, desiredState.angle.getRadians());
        SmartDashboard.putNumber("Angle" + canID, sim.getCasterAngleRad());
        SmartDashboard.putNumber("Speed" + canID, sim.getWheelRadPerSec()*Constants.wheelRadius);
    }
}
