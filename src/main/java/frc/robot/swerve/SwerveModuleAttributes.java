package frc.robot.swerve;

public class SwerveModuleAttributes {
    
    int driveCanID;
    int turnCanID;
    double angleOffset;

    public SwerveModuleAttributes(int driveCanID, int turnCanID, double angleOffset){
        this.driveCanID = driveCanID;
        this.turnCanID = turnCanID;
        this.angleOffset = angleOffset;
    }
}
