package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / (150.0/7); //1 / 15.1 
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.6; // 0.5
        public static final double kITurning = 0;//0.1
        public static final double kDTurning = 0;
    }

    public static final class DriveConstants {
//kTrackWidth and kWheelBase used to be 24.5 
        public static final double kTrackWidth = Units.inchesToMeters(25);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 12;
        public static final int kBackLeftDriveMotorPort = 14;
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kBackRightDriveMotorPort = 20;

        public static final int kFrontLeftTurningMotorPort = 13;
        public static final int kBackLeftTurningMotorPort = 15;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 1;
        //2:47 ALL TRUE
        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;
        //4:28 TTFF
        public static final boolean kFrontLeftDriveEncoderReversed = true; //true on 3/8
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true; // Should be false

        //Note: order was 3, 0, 2, 1
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 1;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //WE NEED TO DO THIS; originally -0.535, -0.565, 4.468, 4.435
        //2/26/24: 21.597, 3.049, 5.734, 1.401
        // 03/07/2024: 1.6697, 3.1962, 6.0057, 1.5323
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.71624; //1.597+0.0727 + 0.1256605-0.216421 + 0.08 + 0.22+3.02; //+0.052444911597363086
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.01928; //3.049+0.1472-0.062838-0.11; //-3.1817416938392014
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 6.01809; //5.734+0.2717-3.11; //+9.229705807278776E-4
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 4.6377107; //1.401+0.1313 -0.0837632 + 0.05586; //+3.113255352392839

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.6*1.2; //taken from Mk4i website:
        //https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777303153
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.5; //Originally / 4
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;
    }
 
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2; //4
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4; //3
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kShooterControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1; //try 7

        public static final double kDeadband = 0.05;
    }
}