// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.auton.AutonTimer;
import frc.robot.commands.auton.TestAutonCommand;
import frc.robot.commands.chassis.RotateDrivetrainByAngle;
import frc.robot.commands.chassis.XDrive;
import frc.robot.commands.conveyor.ReverseInfeedAndConveyor;
import frc.robot.commands.conveyor.RunConveyorOneBall;
import frc.robot.commands.conveyor.RunConveyorTwoBall;
import frc.robot.commands.infeed.RunInfeedSingulatorMotors;
import frc.robot.commands.infeed.ToggleInfeedUp;
import frc.robot.commands.shooter.AcceptLimelightDistance;
import frc.robot.commands.shooter.DecrementShooterIndex;
import frc.robot.commands.shooter.IncrementShooterIndex;
import frc.robot.commands.shooter.ResetDefaultIndex;
import frc.robot.commands.shooter.RunShooterMotorsVbus;
import frc.robot.commands.vision.ToggleCamera;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Limelight;
import frc.robot.utilities.Trajectories;
// import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = DriveSubsystem.get_instance();
  private final Infeed m_singulatorAndInfeed = Infeed.get_instance();
  private final RunInfeedSingulatorMotors runInfeed;
  private static RobotContainer _instance;
  private WaitCommand _wait;
  private static Trajectories _trajectories = Trajectories.get_instance();

  public static final RobotContainer get_instance() {
    if (_instance == null) {
      _instance = new RobotContainer();
    }
    return _instance;
  }
  // private final Shooter m_shooter = Shooter.getInstance();

  // Controller Setup
  private BeakXBoxController m_driverController = new BeakXBoxController(OIConstants.kDriverControllerPort);
  private BeakXBoxController m_operatorController = new BeakXBoxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    AutoConstants.AUTON_THETA_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    // Configure the button bindings
    runInfeed = new RunInfeedSingulatorMotors();
    _wait = new WaitCommand(1.0);
    _wait.addRequirements(m_singulatorAndInfeed);
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                util.deadband(-m_driverController.getLeftYAxis()),
                util.deadband(-m_driverController.getLeftXAxis()),
                util.deadband(-m_driverController.getRightXAxis()),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // ========= OPERATOR CONTROLLER =======
    m_operatorController.a.whenPressed(new RunConveyorTwoBall());
    m_operatorController.b.whenPressed(new RunConveyorOneBall());
    m_operatorController.x.toggleWhenPressed(new RunShooterMotorsVbus());
    m_operatorController.y.toggleWhenPressed(runInfeed);
    m_operatorController.back.toggleWhenPressed(new ReverseInfeedAndConveyor());
    m_operatorController.lb.whenPressed(new DecrementShooterIndex(false));
    m_operatorController.rb.whenPressed(new IncrementShooterIndex(false));
    m_operatorController.lt.whenActive(new DecrementShooterIndex(true));
    m_operatorController.rt.whenActive(new IncrementShooterIndex(true));
    m_operatorController.ls.whenPressed(new ResetDefaultIndex());
    m_operatorController.rs.whenPressed(new AcceptLimelightDistance());
    //====================================

    // ======== DRIVER CONTROLLER ========
    m_driverController.a.whenPressed(new InstantCommand(() -> Limelight.getInstance().toggleLedMode()));
    m_driverController.b.whenPressed(new InstantCommand(() -> m_robotDrive.toggleEnableHoldAngle()));
    m_driverController.x.toggleWhenPressed(new XDrive());
    m_driverController.y.toggleWhenPressed(runInfeed);
    m_driverController.start.whenPressed(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    m_driverController.lb.toggleWhenPressed(new ReverseInfeedAndConveyor());
    m_driverController.rb.whenPressed(new ToggleInfeedUp());
    m_driverController.lt.whileActiveContinuous(runInfeed);
    m_driverController.rs
        .whenPressed(new RotateDrivetrainByAngle(Rotation2d.fromDegrees(Limelight.getInstance().getX()), false));
    m_driverController.ls.whenPressed(new ToggleCamera());
    // ===================================
  }

  public double getRightTrigger() {
    return m_driverController.getRightTrigger();
  }

  /**
   * @param traj Trajectory to follow
   * @return New SwerveControllerCommand - commands DriveSubsystem to follow given
   *         trajectory, then stop
   */
  public Command getSwerveControllerCommand(Trajectory traj) {
    return new SwerveControllerCommand(
        traj,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,
        AutoConstants.AUTON_X_CONTROLLER,
        AutoConstants.AUTON_Y_CONTROLLER,
        AutoConstants.AUTON_THETA_CONTROLLER,
        m_robotDrive::setModuleStates,
        m_robotDrive)
            .andThen(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
    return new TestAutonCommand().deadlineWith(new AutonTimer());
    // return getSwerveControllerCommand(_trajectories.getTestCompFirstBall())
    //     .alongWith(new InstantCommand(() -> Infeed.get_instance().runInfeedSingulatorMotors(1.0)))
    //     .andThen(new RotateDrivetrainByAngle(Rotation2d.fromDegrees(187), true))
    //     .andThen(new RunShooterMotorsVbus().alongWith(new WaitCommand(0.25).andThen(new RunConveyor())).raceWith(new WaitCommand(1.25)))
    //     .andThen(new InstantCommand(() -> Infeed.get_instance().stopInfeedSingulatorMotors()))
    //     .deadlineWith(new AutonTimer());
        // .andThen(getSwerveControllerCommand(_trajectories.getTestCompSecondBall()))
        // .andThen(new WaitCommand(1.5))
        // .andThen(getSwerveControllerCommand(_trajectories.getTestCompReturnShoot()))
        // .andThen(new WaitCommand(2.0))
        // .andThen(new InstantCommand(() -> System.out.println("AUTON FINISHED")));
  }

}
