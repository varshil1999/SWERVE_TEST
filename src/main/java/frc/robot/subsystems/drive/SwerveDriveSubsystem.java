// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2;
// import com.ctre.phoenix.sensors.WPI_Pigeon2;
// import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.*;

public class SwerveDriveSubsystem extends SubsystemBase {
        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */

        public static final double MAX_VOLTAGE = 12.0;
        // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0 *
                        SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
                        SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI);
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));
        // public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =5;

        public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        // private static final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 400); //
        // NavX connected over MXP
        private static final Pigeon2 m_pigeon = new Pigeon2(5);

        private static SwerveModule m_frontLeftModule;
        private static SwerveModule m_frontRightModule;
        private static SwerveModule m_backLeftModule;
        private static SwerveModule m_backRightModule;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
                        getGyroHeading(), new Pose2d(0, 0, new Rotation2d(0.0)));

        public Pose2d m_pose;

        public SwerveDriveSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                // This can either be STANDARD or FAST depending on your gear configuration
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                DriveConstants.FrontLeft.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                DriveConstants.FrontLeft.FRONT_LEFT_MODULE_STEER_MOTOR,
                                DriveConstants.FrontLeft.FRONT_LEFT_MODULE_STEER_ENCODER,
                                DriveConstants.FrontLeft.FRONT_LEFT_MODULE_STEER_OFFSET);

                // We will do the same for the other modules
                m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                DriveConstants.FrontRight.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                DriveConstants.FrontRight.FRONT_RIGHT_MODULE_STEER_MOTOR,
                                DriveConstants.FrontRight.FRONT_RIGHT_MODULE_STEER_ENCODER,
                                DriveConstants.FrontRight.FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                DriveConstants.BackLeft.BACK_LEFT_MODULE_DRIVE_MOTOR,
                                DriveConstants.BackLeft.BACK_LEFT_MODULE_STEER_MOTOR,
                                DriveConstants.BackLeft.BACK_LEFT_MODULE_STEER_ENCODER,
                                DriveConstants.BackLeft.BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                DriveConstants.BackRight.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                DriveConstants.BackRight.BACK_RIGHT_MODULE_STEER_MOTOR,
                                DriveConstants.BackRight.BACK_RIGHT_MODULE_STEER_ENCODER,
                                DriveConstants.BackRight.BACK_RIGHT_MODULE_STEER_OFFSET);
        }

        private Rotation2d getGyroHeading() {
                return Rotation2d.fromDegrees(-m_pigeon.getYaw());
        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                m_pigeon.setYaw(0);
                // m_navx.zeroYaw();
        }

        public double getCurrentYaw() {
                return m_pigeon.getYaw();
        }

        public Rotation2d getGyroscopeRotation() {
                return Rotation2d.fromDegrees(m_pigeon.getYaw());
                // if (m_navx.isMagnetometerCalibrated()) {
                // // We will only get valid fused headings if the magnetometer is calibrated
                // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                // }
                // // // We have to invert the angle of the NavX so that rotating the robot
                // // // counter-clockwise makes the angle increase.
                // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;

                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
                setModuleStates(states);
        }

        public Rotation2d getRotation2d() {
                return Rotation2d.fromDegrees(m_pigeon.getYaw());
        }

        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        public void resetPose() {
                this.m_odometry.resetPosition(this.m_pose, getRotation2d());
        }

        public void resetOdometry(Pose2d pose2d) {
                this.m_odometry.resetPosition(this.m_pose, getGyroscopeRotation());
        }

        @Override
        public void periodic() {
                // SwerveModuleState[] states = new SwerveModuleState[4];
                // states[0] = new SwerveModuleState(
                //         m_frontLeftModule.getDriveVelocity(),
                //         new Rotation2d(m_frontLeftModule.getSteerAngle()));

                

               this.m_pose= m_odometry.update(getGyroHeading(), new SwerveModuleState(
                m_frontLeftModule.getDriveVelocity(),
                new Rotation2d(m_frontLeftModule.getSteerAngle())), 
                new SwerveModuleState(
                m_frontRightModule.getDriveVelocity(),
                new Rotation2d(m_frontRightModule.getSteerAngle())),
                new SwerveModuleState(
                m_backLeftModule.getDriveVelocity(),
                new Rotation2d(m_backLeftModule.getSteerAngle())),
                new SwerveModuleState(
                m_backRightModule.getDriveVelocity(),
                new Rotation2d(m_backRightModule.getSteerAngle()))
                                );

                                
                m_pose = m_odometry.getPoseMeters();

                SmartDashboard.putNumber("Yaw", m_pigeon.getYaw());
                SmartDashboard.putNumber("Left Upper Wheel Power",
                                SwerveDriveSubsystem.m_frontLeftModule.getDriveVelocity());
                SmartDashboard.putNumber("X subsytem", m_pose.getX());
                SmartDashboard.putNumber("Y subsytem", m_pose.getY());
                SmartDashboard.putNumber("abc", m_odometry.getPoseMeters().getRotation().getDegrees());
                SmartDashboard.putNumber("check value", MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND );
                
        }

        public  void setModuleStates(SwerveModuleState[] states) {
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                m_frontLeftModule.set(
                                states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(
                                states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(
                                states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(
                                states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());
        }

        public void stopModules() {
                m_frontLeftModule.set(0, 0);
                m_frontRightModule.set(0, 0);
                m_backLeftModule.set(0, 0);
                m_backRightModule.set(0, 0);
        }

        public Pose2d getPoseTest() {
                return m_pose;
        }

        // public double getCurrentYaw() {
        // return m_navx.getYaw();
        // }
}
