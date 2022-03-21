// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.auton.AutonTimer;
import frc.robot.commands.auton.TestAutonCommand;
import frc.robot.commands.chassis.RotateDrivetrainByLimelightAngle;
import frc.robot.commands.chassis.XDrive;
import frc.robot.commands.climber.HighBarClimb;
import frc.robot.commands.climber.TraversalBarClimb;
import frc.robot.commands.conveyor.ReverseInfeedAndConveyor;
import frc.robot.commands.conveyor.RunConveyor;
import frc.robot.commands.conveyor.RunConveyorOneBall;
import frc.robot.commands.conveyor.RunConveyorTwoBall;
import frc.robot.commands.infeed.RunInfeedSingulatorMotors;
import frc.robot.commands.infeed.ToggleInfeedUp;
import frc.robot.commands.shooter.AcceptLimelightDistance;
import frc.robot.commands.shooter.DecrementShooterIndex;
import frc.robot.commands.shooter.IncrementShooterIndex;
import frc.robot.commands.shooter.ResetDefaultIndex;
import frc.robot.commands.shooter.RunShooterMotors;
import frc.robot.commands.vision.ToggleCamera;
import frc.robot.commands.vision.ToggleLEDMode;
import frc.robot.subsystems.Limelight;
import frc.robot.utilities.Trajectories;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = DriveSubsystem.getInstance();
  private final RunInfeedSingulatorMotors runInfeed;
  private static RobotContainer _instance;
  private static Trajectories _trajectories = Trajectories.getInstance();
  private SendableChooser<Command> _autonChooser = new SendableChooser<Command>();

  public static final RobotContainer getInstance() {
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
    configureButtonBindings();
    //Init Auton Chooser
    initAutonChooser();
    SmartDashboard.putData(_autonChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                m_driverController.getLeftYAxis(),
                -m_driverController.getLeftXAxis(),
                -m_driverController.getRightXAxis(),
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
    m_operatorController.x.toggleWhenPressed(new RunShooterMotors());
    m_operatorController.y.toggleWhenPressed(runInfeed);
    //m_operatorController.start.whenPressed(new HighBarClimb());
    m_operatorController.back.toggleWhenPressed(new ReverseInfeedAndConveyor());
    m_operatorController.lb.whenPressed(new DecrementShooterIndex(false));
    m_operatorController.rb.whenPressed(new IncrementShooterIndex(false));
    m_operatorController.lt.whenActive(new DecrementShooterIndex(true));
    m_operatorController.rt.whenActive(new IncrementShooterIndex(true));
    m_operatorController.ls.whenPressed(new ResetDefaultIndex());
    m_operatorController.rs.whenPressed(new AcceptLimelightDistance());
    m_operatorController.start.toggleWhenPressed(new RunConveyor());
    // ====================================

    // ======== DRIVER CONTROLLER ========
    m_driverController.a.whenPressed(new ToggleLEDMode());
    m_driverController.b.whenPressed(new InstantCommand(() -> m_robotDrive.toggleEnableHoldAngle()));
    m_driverController.x.toggleWhenPressed(new XDrive());
    m_driverController.y.toggleWhenPressed(runInfeed);
    m_driverController.start.whenPressed(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    m_driverController.lb.toggleWhenPressed(new ReverseInfeedAndConveyor());
    m_driverController.rb.whenPressed(new ToggleInfeedUp());
    m_driverController.lt.whileActiveContinuous(runInfeed);
    m_driverController.rs.toggleWhenPressed(new RotateDrivetrainByLimelightAngle());
    m_driverController.ls.whenPressed(new ToggleCamera());
    // ===================================

    // ======== TEMP. CLIMBER CONTROLLER
    BeakXBoxController climberController = new BeakXBoxController(2);
    Climber climber = Climber.getInstance();
    climberController.a.whenPressed(new InstantCommand(() -> climber.toggleTippySolenoid()));
    climberController.lb.whileHeld(new InstantCommand(() -> climber.leftMotorForward(.8)));
    climberController.lb.whenReleased(new InstantCommand(() -> climber.leftMotorOff()));
    climberController.start.whileHeld(new InstantCommand(() -> climber.rightMotorBackward(-.8)));
    climberController.start.whenReleased(new InstantCommand(() -> climber.rightMotorOff()));

    climberController.y.whileHeld(new InstantCommand(() -> climber.leftMotorForward(.8)).alongWith(new InstantCommand(() -> climber.rightMotorForward(.8))));
    climberController.y.whenReleased(new InstantCommand(() -> climber.leftMotorOff()).alongWith(new InstantCommand(() -> climber.rightMotorOff())));
    climberController.x.whileHeld(new InstantCommand(() -> climber.leftMotorBackward(-.8)).alongWith(new InstantCommand(() -> climber.rightMotorBackward(-.8))));
    climberController.x.whenReleased(new InstantCommand(() -> climber.leftMotorOff()).alongWith(new InstantCommand(() -> climber.rightMotorOff())));
    
    climberController.b.whenPressed(new InstantCommand(() -> climber.toggleGrippySolenoid()));
    climberController.rb.whileHeld(new InstantCommand(() -> climber.rightMotorForward(.8)));
    climberController.rb.whenReleased(new InstantCommand(() -> climber.rightMotorOff()));
    climberController.back.whileHeld(new InstantCommand(() -> climber.leftMotorBackward(-.8)));
    climberController.back.whenReleased(new InstantCommand(() -> climber.leftMotorOff()));
    climberController.ls.whenPressed(new TraversalBarClimb());
    climberController.rs.whenPressed(new HighBarClimb());
  }

  public double getRightTrigger() {
    return m_driverController.getRightTrigger();
  }

  public Command getPathPlannerSwerveControllerCommand(PathPlannerTrajectory traj) {
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
  //TODO: Add real Autons because these are just paths rn
  private void initAutonChooser(){
    _autonChooser.setDefaultOption("FourBall_AcquireFirst", getPathPlannerSwerveControllerCommand(_trajectories.FourBall_AcquireFirstCargo()));
    _autonChooser.addOption("FourBall_AcquireLoadingZoneCargo", getPathPlannerSwerveControllerCommand(_trajectories.FourBall_AcquireLoadingZoneCargo()));
    _autonChooser.addOption("FourBall_ReturnToShoot", getPathPlannerSwerveControllerCommand(_trajectories.FourBall_ReturnToShoot()));
    _autonChooser.addOption("FiveBall_AcquireFirstBall", getPathPlannerSwerveControllerCommand(_trajectories.FiveBall_AcquireFirstCargo()));
    _autonChooser.addOption("FiveBall_AcquireSecondCargo", getPathPlannerSwerveControllerCommand(_trajectories.FiveBall_AcquireSecondCargo()));
    _autonChooser.addOption("FiveBall_AcquireLoadingZoneCargo", getPathPlannerSwerveControllerCommand(_trajectories.FiveBall_AcquireLoadingZoneCargo()));
    _autonChooser.addOption("FiveBall_ReturnToShoot", getPathPlannerSwerveControllerCommand(_trajectories.FiveBall_ReturnToShoot()));
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    m_robotDrive.resetOdometry(new Pose2d(_trajectories.FourBall_AcquireFirstCargo().getInitialState().poseMeters.getTranslation(),
    _trajectories.FourBall_AcquireFirstCargo().getInitialState().holonomicRotation));
    return new TestAutonCommand().deadlineWith(new AutonTimer());
    // return getSwerveControllerCommand(_trajectories.getTestCompFirstBall())
    // .alongWith(new InstantCommand(() ->
    // Infeed.getInstance().runInfeedSingulatorMotors(1.0)))
    // .andThen(new RotateDrivetrainByAngle(Rotation2d.fromDegrees(187), true))
    // .andThen(new RunShooterMotorsVbus().alongWith(new
    // WaitCommand(0.25).andThen(new RunConveyor())).raceWith(new
    // WaitCommand(1.25)))
    // .andThen(new InstantCommand(() ->
    // Infeed.getInstance().stopInfeedSingulatorMotors()))
    // .deadlineWith(new AutonTimer());
    // .andThen(getSwerveControllerCommand(_trajectories.getTestCompSecondBall()))
    // .andThen(new WaitCommand(1.5))
    // .andThen(getSwerveControllerCommand(_trajectories.getTestCompReturnShoot()))
    // .andThen(new WaitCommand(2.0))
    // .andThen(new InstantCommand(() -> System.out.println("AUTON FINISHED")));
  }

}
