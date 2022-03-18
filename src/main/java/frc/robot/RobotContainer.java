// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonTimer;
import frc.robot.commands.RotateDrivetrainByAngle;
// import frc.robot.commands.RunConveyorWithEncoder;
// import frc.robot.commands.LiftInfeed;
// import frc.robot.commands.ReverseInfeedAndConveyor;
// import frc.robot.commands.RunConveyorOneBall;
// import frc.robot.commands.RunConveyorTwoBall;
// import frc.robot.commands.RunShooterMotors;
// import frc.robot.commands.ToggleFineAdjustment;
import frc.robot.commands.XDrive;
import frc.robot.subsystems.DriveSubsystem;
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
  private final DriveSubsystem m_robotDrive = DriveSubsystem.get_instance();
  private static RobotContainer _instance;
  private static Trajectories _trajectories = Trajectories.get_instance();
  private SendableChooser<Command> _autonChooser = new SendableChooser<Command>();
  PathPlannerTrajectory _testfirstballacq;

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
    _testfirstballacq = PathPlanner.loadPath("testfirstballacq",
    AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    // Configure the button bindings
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
    //====================================

    // ======== DRIVER CONTROLLER ========
    // m_driverController.back.whenPressed(new InstantCommand(() -> Infeed.get_instance().toggleInfeedUp()));
    m_driverController.right_stick_button
        .whenPressed(new RotateDrivetrainByAngle(Rotation2d.fromDegrees(Limelight.getInstance().getX()), false));
    m_driverController.x.toggleWhenPressed(new XDrive());
    m_driverController.left_bumper.whenPressed(new InstantCommand(() -> m_robotDrive.toggleEnableHoldAngle()));
    m_driverController.start.whenPressed(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    m_driverController.a.whenPressed(new InstantCommand(() -> Limelight.getInstance().toggleLedMode()));
    // ===================================
  }

  public double getRightTrigger() {
    return m_driverController.getRightTrigger();
  }

  public Command getPathPlannerSwerveControllerCommand(PathPlannerTrajectory traj) {
    return new PPSwerveControllerCommand(
        traj,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,
        AutoConstants.AUTON_X_CONTROLLER,
        AutoConstants.AUTON_Y_CONTROLLER,
        AutoConstants.AUTON_THETA_CONTROLLER,
        m_robotDrive::setModuleStates,
        m_robotDrive).deadlineWith(new AutonTimer())
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
    // m_robotDrive.resetOdometry(new Pose2d());
    m_robotDrive.resetOdometry(_testfirstballacq.getInitialPose());
    return _autonChooser.getSelected();
  }

}
