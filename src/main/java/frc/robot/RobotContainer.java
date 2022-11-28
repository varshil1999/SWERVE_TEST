// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
import frc.robot.commands.teleop.drive.SwerveDriveCommand;
// import frc.robot.commands.teleop.shooter.ShooterFlywheelsCommand;
// import frc.robot.commands.teleop.shooter.ShooterHoodDownCommand;
// import frc.robot.commands.teleop.shooter.ShooterHoodUpCommand;
// import frc.robot.commands.teleop.shooter.ShooterTurretDownCommand;
// import frc.robot.commands.teleop.shooter.ShooterTurretUpCommand;
import frc.robot.commands.util.NavxResetCommand;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
// import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ShooterSubsystem shooterSubsystem;
  public final SwerveDriveSubsystem swerveDriveSubsystem;

  // IO devices
  private static Joystick joyS;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.joyS = new Joystick(IOConstants.JoySContants.joyS_ID);
    // this.shooterSubsystem = new ShooterSubsystem();
    this.swerveDriveSubsystem = new SwerveDriveSubsystem();

    // this.shooterSubsystem.setDefaultCommand(
    // new ShooterFlywheelsCommand(this.shooterSubsystem, () ->
    // joyS.getRawAxis(1)));

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    this.swerveDriveSubsystem.setDefaultCommand(new SwerveDriveCommand(
        this.swerveDriveSubsystem,
        () -> -modifyAxis(RobotContainer.joyS.getRawAxis(IOConstants.JoySContants.TRANSLATION_X_AXIS))
            * SwerveDriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(RobotContainer.joyS.getRawAxis(IOConstants.JoySContants.TRANSLATION_Y_AXIS))
            * SwerveDriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(RobotContainer.joyS.getRawAxis(IOConstants.JoySContants.ROTATION_AXIS))
            * SwerveDriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Hood Upward
    // new JoystickButton(RobotContainer.joyS, 5)
    // .toggleWhenPressed(new ShooterHoodUpCommand(this.shooterSubsystem));

    // Hood Downward
    // new JoystickButton(RobotContainer.joyS, 6)
    // .toggleWhenPressed(new ShooterHoodDownCommand(this.shooterSubsystem));

    // Gyro Reset
    new JoystickButton(RobotContainer.joyS, 2)
        // No requirements because we don't need to interrupt anything
        .whenPressed(new NavxResetCommand(this.swerveDriveSubsystem));

    // Turret Upward
    // new JoystickButton(RobotContainer.joyS, 1)
    // .whenPressed(new ShooterTurretUpCommand(this.shooterSubsystem));

    // Turret Downward
    // new JoystickButton(RobotContainer.joyS, 2)
    // .whenPressed(new ShooterTurretDownCommand(this.shooterSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(SwerveDriveSubsystem.m_kinematics);

          
      
    // SmartDashboard.putNumber("pose",
    // swerveDriveSubsystem.getPoseTest().getRotation().getDegrees());

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d()),
        List.of(
        
         new Translation2d(1,0),
         new Translation2d(1.5,0.5),
         new Translation2d(1,1),
         new Translation2d(0.5,0.5),
         new Translation2d(1,0),
         new Translation2d(2,0),
         new Translation2d(2.5, -0.5),
         new Translation2d(2,-1),
         new Translation2d(1.5, -0.5),
         new Translation2d(2,0),
         new Translation2d(1.5,0),
         new Translation2d(1, 0),
         new Translation2d(0.5, 0)
      
        ),
        new Pose2d(0,0,Rotation2d.fromDegrees(0)),
        trajectoryConfig);




//////////WOrking!!!!
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //   new Pose2d(0, 0, new Rotation2d()),
        //   List.of(
        //       new Translation2d(0.5,-0.5),
             
        //       new Translation2d(1, 0),
            
        //       new Translation2d(1.5,0.5),
  
        //       new Translation2d(2,0),
  
        //       new Translation2d(1.5,-0.5),
  
        //       new Translation2d(1,0),
  
        //       new Translation2d(0.5,0.5)
             
             
        //   ),
        //   new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        //   trajectoryConfig);
        



    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        // SwerveDriveSubsystem::getPosition,
        this.swerveDriveSubsystem::getPose,
        SwerveDriveSubsystem.m_kinematics,
        xController,
        yController,
        thetaController,
        this.swerveDriveSubsystem::setModuleStates,
        this.swerveDriveSubsystem);

  

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> swerveDriveSubsystem.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerveDriveSubsystem.stopModules()));

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    // value = Math.copySign(value * value, value);
    value = Math.copySign(0.8 * Math.pow(Math.abs(value), 1.7), value);

    return value;
  }
}
