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
        anglePID = new ProfiledPIDController(Constants.angleKp, Constants.angleKi, Constants.angleKd, new TrapezoidProfile.Constraints(6.0, 6.0));
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        sim = new SwerveModuleHardwareSim();
        canID = CANID;
        SmartDashboard.putData("Drive-"+CANID, drivePID);
        SmartDashboard.putData("Angle-"+CANID, anglePID);
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        desiredState = state;
        TrapezoidProfile.State goal = new TrapezoidProfile.State(MathUtil.angleModulus(desiredState.angle.getRadians()), 0);
        
        // Drive PID units are radians per second. 
        drivePID.setSetpoint(desiredState.speedMetersPerSecond/Constants.wheelRadius); 
        anglePID.setGoal(goal);
    }

    @Override
    public SwerveModuleState getActualState() {
        return new SwerveModuleState(sim.getWheelRadPerSec()* Constants.wheelRadius, new Rotation2d(sim.getCasterAngleRad()));
    }

    @Override
    public SwerveModuleState getDesiredState() {
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
        sim.setCasterMotorVolts(anglePID.calculate(MathUtil.angleModulus(sim.getCasterAngleRad()%2*Math.PI)));

        SmartDashboard.putNumber("Desired Speed" + canID, desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Desired Angle" + canID, MathUtil.angleModulus(desiredState.angle.getRadians()));
        SmartDashboard.putNumber("Angle" + canID, MathUtil.angleModulus(sim.getCasterAngleRad()));
        SmartDashboard.putNumber("Speed" + canID, sim.getWheelRadPerSec()*Constants.wheelRadius);
    }
}
