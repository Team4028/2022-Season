// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AcceptLimelightDistance;
import frc.robot.commands.DecrementShooterIndex;
import frc.robot.commands.IncrementShooterIndex;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.RunConveyor;
import frc.robot.commands.ReverseInfeedAndConveyor;
import frc.robot.commands.RunConveyorOneBall;
import frc.robot.commands.RunConveyorTwoBall;
import frc.robot.commands.RunShooterMotors;
import frc.robot.commands.ToggleAdjustmentStyle;
import frc.robot.commands.ToggleCamera;
import frc.robot.commands.FINDPATHERROR;
import frc.robot.commands.RotateDrivetrainByAngle;
// import frc.robot.commands.RunConveyorWithEncoder;
// import frc.robot.commands.LiftInfeed;
// import frc.robot.commands.ReverseInfeedAndConveyor;
// import frc.robot.commands.RunConveyorOneBall;
// import frc.robot.commands.RunConveyorTwoBall;
// import frc.robot.commands.RunShooterMotors;
// import frc.robot.commands.ToggleFineAdjustment;
import frc.robot.commands.RunInfeedSingulatorMotors;
import frc.robot.commands.RotateDrivetrainByAngle;
import frc.robot.subsystems.Infeed;
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
  private final RunInfeedSingulatorMotors _RunInfeedSingulatorMotors;
  private static RobotContainer _instance;
  private WaitCommand _wait;
  private static Trajectories _trajectories = Trajectories.get_instance();

  private PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  private PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  private ProfiledPIDController thetaController =
  new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  public static final RobotContainer get_instance(){
      if(_instance == null){
          _instance = new RobotContainer();
      }
      return _instance;
  }
//   private final Shooter m_shooter = Shooter.getInstance();




  // Controller Setup
  private BeakXBoxController m_driverController = new BeakXBoxController(OIConstants.kDriverControllerPort);
  private BeakXBoxController m_operatorController = new BeakXBoxController(OIConstants.kOperatorControllerPort);

  //---

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // Configure the button bindings
    _RunInfeedSingulatorMotors = new RunInfeedSingulatorMotors();
    _wait = new WaitCommand(1.0);
    _wait.addRequirements(m_singulatorAndInfeed);
    configureButtonBindings();


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    util.deadband(-m_driverController.getLeftYAxis()),
                    util.deadband(-m_driverController.getLeftXAxis()),
                    util.deadband(-m_driverController.getRightXAxis()),
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
      m_driverController.a.whenPressed(new ToggleCamera());
      m_operatorController.y.toggleWhenPressed(new RunInfeedSingulatorMotors());
      m_operatorController.b.whenPressed(new RunConveyorOneBall());
      m_operatorController.x.toggleWhenPressed(new RunShooterMotors());
      m_operatorController.a.whenPressed(new RunConveyorTwoBall());
      m_operatorController.start.toggleWhenPressed(new RunConveyor());
      m_operatorController.back.toggleWhenPressed(new ReverseInfeedAndConveyor());
      m_operatorController.left_bumper.whenPressed(new DecrementShooterIndex());
      m_operatorController.right_bumper.whenPressed(new IncrementShooterIndex());
      m_operatorController.left_stick_button.whenPressed(new ToggleAdjustmentStyle());
      m_operatorController.right_stick_button.whenPressed(new AcceptLimelightDistance());
// ======== BELOW IMPORTED FROM MK4 CHASSIS ========
      /*m_driverController.left_bumper.whenPressed(new InstantCommand(() -> 
      m_singulatorAndInfeed.liftInfeed())
      .andThen(new WaitCommand(2.0))
      .andThen(new InstantCommand(() -> m_singulatorAndInfeed.holdInfeed())));
      m_driverController.right_bumper.whenPressed(new InstantCommand(() -> 
      m_singulatorAndInfeed.downInfeed())
      .andThen(new WaitCommand(1.0))
      .andThen(new InstantCommand(() -> m_singulatorAndInfeed.holdInfeed())));*/
      m_driverController.y.toggleWhenPressed(_RunInfeedSingulatorMotors);
      m_operatorController.b.whenPressed(new RunConveyor());
      m_operatorController.x.toggleWhenPressed(new RunShooterMotors());
      m_operatorController.a.whenPressed(new RunConveyorTwoBall());
      m_operatorController.start.toggleWhenPressed(new RunConveyorOneBall());
      m_operatorController.back.toggleWhenPressed(new ReverseInfeedAndConveyor());
      m_operatorController.left_bumper.whenPressed(new DecrementShooterIndex());
      m_operatorController.right_bumper.whenPressed(new IncrementShooterIndex());
      m_operatorController.left_stick_button.whenPressed(new ToggleAdjustmentStyle());
      m_operatorController.right_stick_button.whenPressed(new AcceptLimelightDistance());
      // FIXME: bruh spagheti controller
// ======== END IMPORT ======== //
  }

  public double getRightTrigger(){
    return m_driverController.getRightTrigger();
  }
  private Command getSwerveControllerCommand(Trajectory traj){
    return new SwerveControllerCommand(
    traj,
    m_robotDrive::getPose,
    DriveConstants.kDriveKinematics,
    xController,
    yController,
    thetaController,
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
    return getSwerveControllerCommand(_trajectories.getTestCompFirstBall())
    .andThen(new RotateDrivetrainByAngle(Rotation2d.fromDegrees(180), true))
    .andThen(new WaitCommand(2.0))
    .andThen(getSwerveControllerCommand(_trajectories.getTestCompSecondBall()))
    .andThen(new WaitCommand(1.5))
    .andThen(getSwerveControllerCommand(_trajectories.getTestCompReturnShoot()))
    .andThen(new WaitCommand(2.0))
    .andThen(new InstantCommand(() -> System.out.println("AUTON FINISHED")));
  }

}
