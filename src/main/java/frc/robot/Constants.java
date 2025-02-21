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

        public static final double kPDriveReal = 0.05;
        public static final double kIDriveReal = 0;
        public static final double kDDriveReal = 0;

        public static final double kPSteerReal = 8;
        public static final double kISteerReal = 0;
        public static final double kDSteerReal = 0;

        public static final double kPDriveSim = 0.05;
        public static final double kIDriveSim = 0;
        public static final double kDDriveSim = 0;

        public static final double kPSteerSim = 8;
        public static final double kISteerSim = 0;
        public static final double kDSteerSim = 0;

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
}