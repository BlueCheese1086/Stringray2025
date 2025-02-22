package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

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
        
        public static final int DT_FLEncoder = 11; // CAN
        public static final int DT_FREncoder = 12; // CAN
        public static final int DT_BLEncoder = 13; // CAN
        public static final int DT_BREncoder = 14; // CAN

        public static final int GYRO_Pigeon2Id = 15; // CAN

        public static final int ELEV_LeftId = 31; // CAN
        public static final int ELEV_RightId = 32; // CAN
    }

    public class PIDValues {
        public static double kPDrive = DriveConstants.kPDriveDefault;
        public static double kIDrive = DriveConstants.kIDriveDefault;
        public static double kDDrive = DriveConstants.kDDriveDefault;

        public static double kPSteer = DriveConstants.kPSteerDefault;
        public static double kISteer = DriveConstants.kISteerDefault;
        public static double kDSteer = DriveConstants.kDSteerDefault;

        public static double kPElev = ElevatorConstants.kPDefault;
        public static double kIElev = ElevatorConstants.kIDefault;
        public static double kDElev = ElevatorConstants.kDDefault;
    }

    public class DriveConstants {
        public static final Distance robotWidth = Inches.of(24);
        public static final Distance robotLength = Inches.of(24);

        public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(6);
        public static final LinearAcceleration maxLinearAcceleration = MetersPerSecondPerSecond.of(3);
        public static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(3);
        public static final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(2);

        public static final Mass robotMass = Pounds.of(160);
        public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(0.02);

        public static final Distance wheelRadius = Inches.of(4);

        public static final Current driveCurrentLimit = Amps.of(60);
        public static final Current steerCurrentLimit = Amps.of(30);

        public static final double kPDriveDefault = 0.05;
        public static final double kIDriveDefault = 0;
        public static final double kDDriveDefault = 0;

        public static final double kPSteerDefault = 8;
        public static final double kISteerDefault = 0;
        public static final double kDSteerDefault = 0;

        // TODO Find using SysID
        // public static final double kVDrive = 0;
        // public static final double kADrive = 0;

        public static final double driveGearRatio = 5.14;
        public static final double steerGearRatio = 12.8;

        public static final double driveMOI = 0.025;
        public static final double steerMOI = 0.004;

        public static final double metersPerRotation = wheelRadius.in(Meters) / driveGearRatio;

        public static final Translation2d flModuleOffset = new Translation2d(robotWidth.div( 2), robotLength.div( 2));
        public static final Translation2d frModuleOffset = new Translation2d(robotWidth.div( 2), robotLength.div(-2));
        public static final Translation2d blModuleOffset = new Translation2d(robotWidth.div(-2), robotLength.div( 2));
        public static final Translation2d brModuleOffset = new Translation2d(robotWidth.div(-2), robotLength.div(-2));

        public static final double flEncoderOffset = 4.28; // Radians
        public static final double frEncoderOffset = 1.97; // Radians
        public static final double blEncoderOffset = 4.06; // Radians
        public static final double brEncoderOffset = 6.52; // Radians

        // Arrays for easy configuration access
        public static final Translation2d[] translations = { flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset };

        public static final double[][] moduleConfigs = {
            {RobotMap.DT_FLDrive, RobotMap.DT_FLSteer, RobotMap.DT_FLEncoder, DriveConstants.flEncoderOffset}, // FL: drive id, steer id, encoder id, encoder offset
            {RobotMap.DT_FRDrive, RobotMap.DT_FRSteer, RobotMap.DT_FREncoder, DriveConstants.frEncoderOffset}, // FR: drive id, steer id, encoder id, encoder offset
            {RobotMap.DT_BLDrive, RobotMap.DT_BLSteer, RobotMap.DT_BLEncoder, DriveConstants.blEncoderOffset}, // BL: drive id, steer id, encoder id, encoder offset
            {RobotMap.DT_BRDrive, RobotMap.DT_BRSteer, RobotMap.DT_BREncoder, DriveConstants.brEncoderOffset}  // BR: drive id, steer id, encoder id, encoder offset
        };
    }

    public class CarriageConstants {
        public static final int CARRIAGE_MOTOR_ID = 21;
        public static final double maxPercent = 1;
    }

    public class ElevatorConstants {
        public static final Mass mass = Pounds.of(30.0);
        public static final Distance radius = Inches.of(1.025);
        public static final Distance maxHeight = Meters.of(10);
        public static final double gearRatio = 5.0;

        public static final double positionConversionFactor = 2.0 * Math.PI * radius.in(Meters) / gearRatio;
        public static final double velocityConversionFactor = positionConversionFactor / 60.0;

        public static final double kPDefault = 0;
        public static final double kIDefault = 0;
        public static final double kDDefault = 0;

        public static final double kS = 0.27117;
        public static final double kG = 0.42059;
        public static final double kV = 6.049;
        public static final double kA = 0.75;

        public static final double maxProfileVoltage = 6.0;

        public static final Distance sysIdMinPosition = Meters.of(0.1);
        public static final Distance sysIdMaxPosition = Meters.of(1.5);

        public static final double sysIdRampUp = 2.5;
        public static final double sysIdStep = 5.5;
        public static final double sysIdTimeout = 20.0;
    }
}