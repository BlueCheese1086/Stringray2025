package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final boolean isReplay = false;

    public static final Angle TipThreshold = Degrees.of(6);
    public static final double TipTimeout = 1;

    public static final double PrecisionScalar = 0.75;

    public class RobotMap {
        public static final int DT_FLDrive = 1; // CAN
        public static final int DT_FLSteer = 2; // CAN
        public static final int DT_FRDrive = 3; // CAN
        public static final int DT_FRSteer = 4; // CAN
        public static final int DT_BLDrive = 5; // CAN
        public static final int DT_BLSteer = 6; // CAN
        public static final int DT_BRDrive = 7; // CAN
        public static final int DT_BRSteer = 8; // CAN

        public static final int DT_FLEncoder = 2; // CAN
        public static final int DT_FREncoder = 4; // CAN
        public static final int DT_BLEncoder = 6; // CAN
        public static final int DT_BREncoder = 8; // CAN

        public static final int GYRO_Pigeon2Id = 0; // CAN

        public static final int ELEV_LeftId = 10; // CAN
        public static final int ELEV_RightId = 11; // CAN

        public static final int CARRIAGE_CoralId = 20; // CAN
        public static final int CARRIAGE_CoralLaserId = 21; // CAN (CANandColor)

        public static final int CARRIAGE_TrackId = 30; // CAN
        public static final int CARRIAGE_TrackLaserId = 31; // CAN (LaserCAN)

        public static final int CARRIAGE_AlgaeId = 41; // CAN
        public static final int CARRIAGE_AlgaeLaserId = 42; // CAN (LaserCAN)

        public static final int CLIMB_MotorId = 50; // CAN
    }
    
    public class Poses {
        public static final Pose2d REEF_Side1Left  = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(3.106, 4.175, Rotation2d.fromDegrees(  0)) : new Pose2d(14.442, 4.175, Rotation2d.fromDegrees(180));
        public static final Pose2d REEF_Side1Right = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(3.106, 3.847, Rotation2d.fromDegrees(  0)) : new Pose2d(14.442, 3.847, Rotation2d.fromDegrees(180));
        public static final Pose2d REEF_Side2Left  = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(3.926, 5.350, Rotation2d.fromDegrees(300)) : new Pose2d(14.622, 5.350, Rotation2d.fromDegrees(120));
        public static final Pose2d REEF_Side2Right = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(3.675, 5.160, Rotation2d.fromDegrees(300)) : new Pose2d(13.873, 5.160, Rotation2d.fromDegrees(120));
        public static final Pose2d REEF_Side3Left  = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(5.374, 5.160, Rotation2d.fromDegrees(240)) : new Pose2d(12.174, 5.160, Rotation2d.fromDegrees( 60));
        public static final Pose2d REEF_Side3Right = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(5.066, 5.350, Rotation2d.fromDegrees(240)) : new Pose2d(12.482, 5.350, Rotation2d.fromDegrees( 60));
        public static final Pose2d REEF_Side4Left  = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(5.915, 3.870, Rotation2d.fromDegrees(180)) : new Pose2d(11.633, 3.870, Rotation2d.fromDegrees(  0));
        public static final Pose2d REEF_Side4Right = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(5.915, 4.175, Rotation2d.fromDegrees(180)) : new Pose2d(11.633, 4.175, Rotation2d.fromDegrees(  0));
        public static final Pose2d REEF_Side5Left  = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(5.066, 2.699, Rotation2d.fromDegrees(120)) : new Pose2d(12.482, 2.699, Rotation2d.fromDegrees(300));
        public static final Pose2d REEF_Side5Right = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(5.374, 2.867, Rotation2d.fromDegrees(120)) : new Pose2d(12.174, 2.867, Rotation2d.fromDegrees(300));
        public static final Pose2d REEF_Side6Left  = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(3.675, 2.867, Rotation2d.fromDegrees( 60)) : new Pose2d(13.873, 2.867, Rotation2d.fromDegrees(240));
        public static final Pose2d REEF_Side6Right = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(3.926, 2.669, Rotation2d.fromDegrees( 60)) : new Pose2d(13.622, 2.669, Rotation2d.fromDegrees(240));

        public static final Pose2d PROCESSOR_Top_Left   = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(0.749, 6.570, Rotation2d.fromDegrees(305)) : new Pose2d(16.799, 6.570, Rotation2d.fromDegrees(-125));
        public static final Pose2d PROCESSOR_Top_Center = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(1.134, 6.899, Rotation2d.fromDegrees(305)) : new Pose2d(16.414, 6.899, Rotation2d.fromDegrees(-125));
        public static final Pose2d PROCESSOR_Top_Right  = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(1.639, 7.247, Rotation2d.fromDegrees(305)) : new Pose2d(15.909, 7.247, Rotation2d.fromDegrees(-125));

        public static final Pose2d PROCESSOR_Bottom_Left   = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(0.749, 1.685, Rotation2d.fromDegrees(55)) : new Pose2d(16.799, 1.685, Rotation2d.fromDegrees(125));
        public static final Pose2d PROCESSOR_Bottom_Center = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(1.134, 1.081, Rotation2d.fromDegrees(55)) : new Pose2d(16.414, 1.081, Rotation2d.fromDegrees(125));
        public static final Pose2d PROCESSOR_Bottom_Right  = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? new Pose2d(1.639, 1.397, Rotation2d.fromDegrees(55)) : new Pose2d(15.909, 1.397, Rotation2d.fromDegrees(125));
    }

    public class CarriageConstants {
        public static final double proximityThreshold = 0.9;
        
    }

}
