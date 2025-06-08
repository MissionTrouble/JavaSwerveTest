package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelCircumference = kWheelDiameterMeters*Math.PI;
        public static final double kDriveMotorGearRatio = 6.75; //mk4 L2
        public static final double kTurningMotorGearRatio = 12.8;
        public static final double kDriveEncoderRot2Meter = (Math.PI * kWheelDiameterMeters)/kDriveMotorGearRatio;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRot2Rad = (2.0 * Math.PI)/kTurningMotorGearRatio;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPModuleTurningController = 0.5;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(21);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),//2
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),//4
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),//1
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));//3

        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 7;
        public static final int kBackLeftTurningMotorPort = 1;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 3;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final boolean kFrontLeftAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightAbsoluteEncoderReversed = false;
        public static final boolean kBackRightAbsoluteEncoderReversed = false;

        public static final double kFrontLeftAbsoluteEncoderOffsetRad = -0.254;
        public static final double kBackLeftAbsoluteEncoderOffsetRad = -1.252;
        public static final double kFrontRightAbsoluteEncoderOffsetRad = -1.816;
        public static final double kBackRightAbsoluteEncoderOffsetRad = -4.811;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        public static final Boolean kDriverFieldOriented = true;

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;

        public static final double kDeadband = 0.05;
    }
    
    public static final class SimConstants {
        public static final double kNeoFreeRpm = 5676.0;
        public static final double kRobotMassKg = 45;        // incl. bumpers+battery
        public static final double kMoI          = 7.5;      // rough moment of inertia
    }
    
}