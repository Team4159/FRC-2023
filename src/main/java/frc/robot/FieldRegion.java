package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.VisionConstants;

public class FieldRegion {
    private Alliance alliance;
    private int gridIndex;
    
    private FieldRegion(Alliance alliance, int gridIndex) {
        this.alliance = alliance;
        this.gridIndex= alliance.equals(Alliance.Invalid) ? -1 : gridIndex;
    }

    public static FieldRegion lookup(Pose2d pose) { // finds the scoring region that the pose parameter is currently in
        double x = pose.getX();
        double y = pose.getY();

        Alliance alliance = x <= 2.92 ? Alliance.Blue : x >= 13.62 ? Alliance.Red : Alliance.Invalid;
        int index = -1;

        if (y >= 4.71 && y <= 5.50)
            index = 9;
        else if (y >= 4.15)
            index = 8;
        else if (y >= 3.59)
            index = 7;
        else if (y >= 3.03)
            index = 6;
        else if (y >= 2.47)
            index = 5;
        else if (y >= 1.91)
            index = 4;
        else if (y >= 1.35)
            index = 3;
        else if (y >= 0.79)
            index = 2;
        else if (y >= 0)
            index = 1;
        
        return new FieldRegion(alliance, index);
    }

    private static final double[] gridIndexY = {
        0.0d, 0.51, 1.07, 1.63, 2.19, 2.75, 3.31, 3.87, 4.43, 4.99
    };
    public Pose2d getTargetPose() { // returns the closest pose for scoring at the grid
        if (this.alliance.equals(Alliance.Invalid) || this.gridIndex <= 0) return null;
        return new Pose2d(
            this.alliance.equals(Alliance.Blue) ? 1.84 : VisionConstants.fieldWidth-1.84,
            gridIndexY[this.gridIndex],
            Rotation2d.fromDegrees(this.alliance.equals(Alliance.Blue) ? 180 : 0)
        );
    }

    public Pose2d getChargePose() { // returns the closest pose for scoring at the grid
        if (this.alliance.equals(Alliance.Invalid)) return null;
        return new Pose2d(
            this.alliance.equals(Alliance.Blue) ? 3.89 : 12.65,
            gridIndexY[5],
            Rotation2d.fromDegrees(this.alliance.equals(Alliance.Blue) ? 180 : 0)
        );
    }

    /* NO DELETE >:( This took like four hours
     * 
     * 
     * Full width of coordinate system with origin at april tag origin: 651.22in = 16.54m
     * 
     * 
     * Region names:
     * 
     * Cone     B9                      R9     4.99
     * Cube     B8                      R8     4.43
     * Cone     B7                      R7     3.87
     * Cone     B6                      R6     3.31
     * Cube     B5   BC     M    RC     R5     2.75
     * Cone     B4                      R4     2.19
     * Cone     B3                      R3     1.63
     * Cube     B2                      R2     1.07
     * Cone     B1                      R1     0.51
     */
}
