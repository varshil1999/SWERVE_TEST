// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.util.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final class FrontLeft {
            // This is the ID of the drive motor
            public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 46;
            // This is the ID of the steer motor
            public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 45;
            // This is the ID of the steer encoder
            public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3;
            // This is how much the steer encoder is offset from true zero (In our case,
            // zero is facing straight forward)
            public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(307.529);
        }

        public static final class FrontRight {
            public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 48;
            public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 47;
            public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 4;
            public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(187.910);

        }

        public static final class BackLeft {
            public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 44;
            public static final int BACK_LEFT_MODULE_STEER_MOTOR = 43;
            public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2;
            public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(75.586);

        }

        public static final class BackRight {
            public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 42;
            public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 41;
            public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 1;
            public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(56.162);

        }

        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.61665;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.61665;
        // public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.1;
        // public static final double MAX_VOLTAGE = 6;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 11.39;

    }

    // public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    // public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2.9;
    public static final double kMaxAngularSpeedRadiansPerSecond =

    SwerveDriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.9;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared =
    Math.PI / 4;
    public static final double kPXController = 0.4;
    public static final double kPYController = 0.4;
    public static final double kPThetaController = 0.6;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints
    = //
    new TrapezoidProfile.Constraints(
    kMaxAngularSpeedRadiansPerSecond,
    kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    // public static final class AutoConstants {
    //     public static final double kMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    //     public static final double kMaxAngularSpeedRadiansPerSecond = //
    //            kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    //     public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    //     public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    //     public static final double kPXController = 1.5;
    //     public static final double kPYController = 1.5;
    //     public static final double kPThetaController = 3;


    //     public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
    //             new TrapezoidProfile.Constraints(
    //                     kMaxAngularSpeedRadiansPerSecond,
    //                     kMaxAngularAccelerationRadiansPerSecondSquared);
    // }

    public static class ShooterConstants {

        public static class TurretConstants {
            public static int turretPort = 9;
            public static double turretGearRatio = 34799;
            public static Gains kGains2 = new Gains(0.3, 0, 0, 0, 0, 0.5);

            public static double angularPositionIncrement(double angle) {
                return (angle / 360) * TurretConstants.turretGearRatio;
            }
        }

        public static class HoodConstants {
            public static int hoodPort = 7;
            public static double hoodGearRatio = 107520;
            public static Gains kGains1 = new Gains(0.3, 0, 0, 0, 0, 1.0);

            public static double angularPositionIncrement(double angle) {
                return (angle / 360) * HoodConstants.hoodGearRatio;
            }
        }

        public static class FlywheelConstants {
            public static int flywheelsPort = 4;
            public static double flywheelMaxSpeed = 0.2;
            public static Gains kGains = new Gains(0.09, 0.000, 0.00, 0.046, 0, 1.0);
        }

        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public static final boolean kSensorPhase = true;

        public static double shooterDeadband = 0.15;

        public static boolean kMotorInvert = false;

        /**
         * Gains used in Positon Closed Loop, to be adjusted accordingly
         * Gains(kp, ki, kd, kf, izone, peak output);
         */

        public static Gains kGains3 = new Gains(0.3, 0.00, 0.00, 0.00, 0, 1.0);
        public static Gains kGains2 = new Gains(0.2, 0.0, 0.0, 0.0, 0, 1.0);

    }

    public static final class IOConstants {
        public static double NAVX_RESET_THRESHOLD = 0.2;

        public static final class JoySContants {

            public static final int joyS_ID = 0;
            public static final int TRANSLATION_X_AXIS = 1;
            public static final int TRANSLATION_Y_AXIS = 0;
            public static final int ROTATION_AXIS = 4;

        }
    }
}
