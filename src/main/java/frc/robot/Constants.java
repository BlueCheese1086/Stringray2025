package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Constants {
    public static final boolean isReplay = false;

    public class RobotMap {
        public static final int DT_FLDrive = 1; // CAN
        public static final int DT_FLSteer = 2; // CAN
        public static final int DT_FRDrive = 3; // CAN
        public static final int DT_FRSteer = 4; // CAN
        public static final int DT_BLDrive = 5; // CAN
        public static final int DT_BLSteer = 6; // CAN
        public static final int DT_BRDrive = 7; // CAN
        public static final int DT_BRSteer = 8; // CAN

        public static final int DT_FLEncoder = 0; // CAN
        public static final int DT_FREncoder = 1; // CAN
        public static final int DT_BLEncoder = 2; // CAN
        public static final int DT_BREncoder = 3; // CAN

        public static final int GYRO_Pigeon2Id = 0; // CAN

        public static final int CARRIAGE_MotorId = 20; // CAN
        public static final int CARRIAGE_CandandColorId = 21; // CAN

        public static final int ELEV_LeftId = 10; // CAN
        public static final int ELEV_RightId = 11; // CAN
    }

    public class DriveConstants {
        public static final Distance robotWidth = Inches.of(22.75);
        public static final Distance robotLength = Inches.of(22.75);

        public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(4.73);
        public static final LinearAcceleration maxLinearAcceleration = MetersPerSecondPerSecond.of(3);
        public static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(3 * Math.PI);
        public static final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(Math.PI);

        public static final Mass robotMass = Kilograms.of(50);
        public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(6.8);

        public static final Distance wheelRadius = Inches.of(1.931);

        public static final Current driveCurrentLimit = Amps.of(60);
        public static final Current steerCurrentLimit = Amps.of(30);

        public static final double kPDriveDefault = RobotBase.isReal() ? 1       : 0.3;
        public static final double kIDriveDefault = RobotBase.isReal() ? 0       : 0;
        public static final double kDDriveDefault = RobotBase.isReal() ? 0       : 0.01;
        public static final double kSDriveDefault = RobotBase.isReal() ? 0.06241 : 0;
        public static final double kVDriveDefault = RobotBase.isReal() ? 0.30278 : 0;
        public static final double kADriveDefault = RobotBase.isReal() ? 0       : 0;

        public static final double kPSteerDefault = RobotBase.isReal() ? 1    : 5;
        public static final double kISteerDefault = RobotBase.isReal() ? 0    : 0;
        public static final double kDSteerDefault = RobotBase.isReal() ? 0.5  : 0;
        public static final double kSSteerDefault = RobotBase.isReal() ? 0.1  : 0;
        public static final double kVSteerDefault = RobotBase.isReal() ? 2.66 : 0;
        public static final double kASteerDefault = RobotBase.isReal() ? 0    : 0;

        public static final double driveGearRatio = 5.14;
        public static final double steerGearRatio = 12.8;

        public static final double driveMOI = 0.025;
        public static final double steerMOI = 0.004;

        public static final Translation2d flModuleOffset = new Translation2d(robotWidth.div( 2), robotLength.div( 2));
        public static final Translation2d frModuleOffset = new Translation2d(robotWidth.div( 2), robotLength.div(-2));
        public static final Translation2d blModuleOffset = new Translation2d(robotWidth.div(-2), robotLength.div( 2));
        public static final Translation2d brModuleOffset = new Translation2d(robotWidth.div(-2), robotLength.div(-2));

        public static final double flEncoderOffset = 0.566894531; // Rotations
        public static final double frEncoderOffset = 0.388427734; // Rotations
        public static final double blEncoderOffset = 0.114257813; // Rotations
        public static final double brEncoderOffset = 0.085205078; // Rotations

        // Arrays for easy configuration access
        public static final Translation2d[] translations = { flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset };

        public static final double[][] moduleConfigs = {
            {RobotMap.DT_FLDrive, RobotMap.DT_FLSteer, RobotMap.DT_FLEncoder, DriveConstants.flEncoderOffset}, // FL: drive id, steer id, encoder id, encoder offset
            {RobotMap.DT_FRDrive, RobotMap.DT_FRSteer, RobotMap.DT_FREncoder, DriveConstants.frEncoderOffset}, // FR: drive id, steer id, encoder id, encoder offset
            {RobotMap.DT_BLDrive, RobotMap.DT_BLSteer, RobotMap.DT_BLEncoder, DriveConstants.blEncoderOffset}, // BL: drive id, steer id, encoder id, encoder offset
            {RobotMap.DT_BRDrive, RobotMap.DT_BRSteer, RobotMap.DT_BREncoder, DriveConstants.brEncoderOffset}  // BR: drive id, steer id, encoder id, encoder offset
        };

        public static final SwerveModuleState[] xStates = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees( 135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees( 135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-135))
        };

        public static final double kPXControllerDefault = 10;
        public static final double kIXControllerDefault = 0;
        public static final double kDXControllerDefault = 0;

        public static final double kPYControllerDefault = 10;
        public static final double kIYControllerDefault = 0;
        public static final double kDYControllerDefault = 0;
        
        public static final double kPThetaControllerDefault = 7;
        public static final double kIThetaControllerDefault = 0;
        public static final double kDThetaControllerDefault = 0;
    }

    public class ElevatorConstants {
        public static final Mass mass = Pounds.of(25.1);
        public static final Distance radius = Meters.of(0.02864789);
        public static final Distance maxHeight = Inches.of(69);
        public static final double gearRatio = 9;

        public static final LinearVelocity maxVelocity = MetersPerSecond.of(1.8);
        public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(6.5);

        public static final double positionConversionFactor = 2.0 * Math.PI * radius.in(Meters) / gearRatio;
        public static final double velocityConversionFactor = positionConversionFactor / 60.0;

        public static final double kPDefault = 40;
        public static final double kIDefault = 0;
        public static final double kDDefault = 0;

        public static final double[] kSDefaults = {0.6, 0.6, 0.9};
        public static final double[] kGDefaults = {0.3, 0.3, 0.3};
        public static final double kVDefault = 3;
        public static final double[] kADefaults = {0, 0, 0};

        public static final double maxProfileVoltage = 6.0;

        public static final Distance sysIdMinPosition = Meters.of(0.1);
        public static final Distance sysIdMaxPosition = Meters.of(1.5);

        public static final double sysIdRampUp = 2.5;
        public static final double sysIdStep = 5.5;
        public static final double sysIdTimeout = 20.0;
    }

    public class VisionConstants {
        public static final String lCameraName = "Camera_Module_v1_l";
        public static final String rCameraName = "Camera_Module_v1_r";

        public static final Transform3d lCameraTransform = new Transform3d(Inches.of(8.410427), Inches.of(11.276584), Inches.of(8.209095), new Rotation3d(Degrees.zero(), Degrees.of(-36.053760), Degrees.of( 20)));
        public static final Transform3d rCameraTransform = new Transform3d(Inches.of(8.410427), Inches.of(11.276584), Inches.of(8.209095), new Rotation3d(Degrees.zero(), Degrees.of(-36.053760), Degrees.of(-20)));

        public static final PoseStrategy strategy = PoseStrategy.LOWEST_AMBIGUITY;
        public static final AprilTagFields field = AprilTagFields.kDefaultField;

        public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(2, 2, 8);
        public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.1, 0.1, 1);
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
}
