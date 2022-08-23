package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveModuleState {

    public void setDesiredState(SwerveModuleState state);
    public SwerveModuleState getModuleState();
    public SwerveModuleState getDesiredState();
    public void resetEncoders();
    public void update();
}
