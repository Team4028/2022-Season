// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AcceptLimelightDistance;
import frc.robot.commands.DecrementShooterIndex;
import frc.robot.commands.IncrementShooterIndex;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.RunConveyorWithEncoder;
import frc.robot.commands.ReverseInfeedAndConveyor;
import frc.robot.commands.RunConveyorOneBall;
import frc.robot.commands.RunConveyorTwoBall;
import frc.robot.commands.RunShooterMotors;
import frc.robot.commands.ToggleFineAdjustment;
import frc.robot.commands.RunInfeedSingulatorMotors;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = DriveSubsystem.get_instance();
  private final Infeed m_infeed = Infeed.get_instance();
  private final Shooter m_shooter = Shooter.getInstance();

  // Controller Setup
  private BeakXBoxController m_driverController = new BeakXBoxController(OIConstants.kDriverControllerPort);
  private BeakXBoxController m_operatorController = new BeakXBoxController(OIConstants.kOperatorControllerPort);
  //---

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    util.deadband(-m_driverController.getLeftYAxis()/2),
                    util.deadband(-m_driverController.getLeftXAxis()/2),
                    util.deadband(-m_driverController.getRightXAxis()/2),
                    true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
      m_driverController.start.whenPressed(new InstantCommand(() -> m_robotDrive.zeroHeading()));
      m_operatorController.y.toggleWhenPressed(new RunInfeedSingulatorMotors());
      m_operatorController.b.whenPressed(new RunConveyorWithEncoder());
      m_operatorController.x.toggleWhenPressed(new RunShooterMotors());
      m_operatorController.a.whenPressed(new RunConveyorTwoBall());
      m_operatorController.start.toggleWhenPressed(new RunConveyorOneBall());
      m_operatorController.right_bumper.whenPressed(new InstantCommand(() -> m_shooter.shiftShooterVbus(0, 0.02)));
      m_operatorController.left_bumper.whenPressed(new InstantCommand(() -> m_shooter.shiftShooterVbus(0.02, 0)));
      m_operatorController.back.toggleWhenPressed(new ReverseInfeedAndConveyor());
      m_operatorController.left_bumper.whenPressed(new DecrementShooterIndex());
      m_operatorController.right_bumper.whenPressed(new IncrementShooterIndex());
      m_operatorController.left_stick_button.whenPressed(new ToggleFineAdjustment());
      m_operatorController.right_stick_button.whenPressed(new AcceptLimelightDistance());
      // FIXME: bruh spagheti controller

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
