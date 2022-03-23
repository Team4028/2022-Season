// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.auton.AutonTimer;
import frc.robot.commands.auton.FiveBallAuton;
import frc.robot.commands.auton.FourBallAuton;
import frc.robot.commands.chassis.RotateDrivetrainByLimelightAngle;
import frc.robot.commands.chassis.RotateDrivetrainToAngle;
import frc.robot.commands.chassis.XDrive;
import frc.robot.commands.climber.HighBarClimb;
import frc.robot.commands.climber.MoveArm;
import frc.robot.commands.climber.TraversalBarClimb;
import frc.robot.commands.conveyor.ReverseInfeedAndConveyor;
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
                getSpeedScaledDriverLeftY(),
                getSpeedScaledDriverLeftX(),
                getSpeedScaledDriverRightX(),
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
    m_operatorController.start.toggleWhenPressed(new ToggleInfeedUp());
    // ====================================

    // ======== DRIVER CONTROLLER ========
    m_driverController.a.whenPressed(new ToggleLEDMode());
    m_driverController.x.toggleWhenPressed(new XDrive());
    m_driverController.y.toggleWhenPressed(runInfeed);
    m_driverController.start.whenPressed(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    m_driverController.lb.toggleWhenPressed(new ReverseInfeedAndConveyor());
    m_driverController.rb.whenPressed(new ToggleInfeedUp());
    m_driverController.lt.whileActiveContinuous(runInfeed);
    m_driverController.rs.toggleWhenPressed(new RotateDrivetrainByLimelightAngle().withTimeout(2.0));
    m_driverController.ls.whenPressed(new ToggleCamera());
    m_driverController.b.toggleWhenPressed(new RotateDrivetrainToAngle(Rotation2d.fromDegrees(43)));
    // ===================================

    // ======== TEMP. CLIMBER CONTROLLER
    // BeakXBoxController climberController = new BeakXBoxController(2);
    // Climber climber = Climber.getInstance();
    // // climberController.a.whenPressed(new InstantCommand(() -> climber.toggleTippySolenoid()));
    // climberController.lb.whileHeld(new InstantCommand(() -> climber.leftMotorForward(.3)));
    // climberController.lb.whenReleased(new InstantCommand(() -> climber.leftMotorOff()));
    // climberController.start.whileHeld(new InstantCommand(() -> climber.rightMotorBackward(-.8)));
    // climberController.start.whenReleased(new InstantCommand(() -> climber.rightMotorOff()));

    // climberController.y.whileHeld(new InstantCommand(() -> climber.leftMotorForward(.8)).alongWith(new InstantCommand(() -> climber.rightMotorForward(.8))));
    // climberController.y.whenReleased(new InstantCommand(() -> climber.leftMotorOff()).alongWith(new InstantCommand(() -> climber.rightMotorOff())));
    // climberController.x.whileHeld(new InstantCommand(() -> climber.leftMotorBackward(-.8)).alongWith(new InstantCommand(() -> climber.rightMotorBackward(-.8))));
    // climberController.x.whenReleased(new InstantCommand(() -> climber.leftMotorOff()).alongWith(new InstantCommand(() -> climber.rightMotorOff())));
    
    // climberController.b.whenPressed(new InstantCommand(() -> climber.toggleGrippySolenoid()));
    // climberController.rb.whileHeld(new InstantCommand(() -> climber.rightMotorForward(.8)));
    // climberController.rb.whenReleased(new InstantCommand(() -> climber.rightMotorOff()));
    // climberController.back.whileHeld(new InstantCommand(() -> climber.leftMotorBackward(-.8)));
    // climberController.back.whenReleased(new InstantCommand(() -> climber.leftMotorOff()));

    // climberController.a.whenPressed(new MoveArm(0.8, 104));
    // climberController.ls.whenPressed(new TraversalBarClimb());
    // climberController.rs.whenPressed(new HighBarClimb());

    // climberController.lt.whenPressed(new InstantCommand(() -> climber.slowDrop()));
    // climberController.rt.whenPressed(new InstantCommand(() -> climber.slowUp()));

    // // ======= BRUH PIT CONTROLLER
    // BeakXBoxController pitController = new BeakXBoxController(3);

    // pitController.a.whenPressed(new InstantCommand(() -> climber.resetEncoders()));
    // pitController.start.whenPressed(new MoveArm(.2, 0));
    // pitController.back.whenPressed(new MoveArm(-.2, -35));

    // pitController.lb.whenPressed(new InstantCommand(() -> climber.setLeftEncoder(EncoderConstants.kClimberLeftStart)));
    // pitController.rb.whenPressed(new InstantCommand(() -> climber.setRightEncoder(EncoderConstants.kClimberRightStart)));
  }

  public double getRightTrigger() {
    return m_driverController.getRightTrigger();
  }
  public double getSpeedScaledDriverRightX(){
    return (DriveConstants.BASE_SPEED_SCALE + getRightTrigger() * (1.0 - DriveConstants.BASE_SPEED_SCALE)) * -util.deadband(m_driverController.getRightXAxis());
  }
  public double getSpeedScaledDriverLeftX(){
    return (DriveConstants.BASE_SPEED_SCALE + getRightTrigger() * (1.0 - DriveConstants.BASE_SPEED_SCALE)) * -util.deadband(m_driverController.getLeftXAxis());
  }
  public double getSpeedScaledDriverLeftY(){
    return (DriveConstants.BASE_SPEED_SCALE + getRightTrigger() * (1.0 - DriveConstants.BASE_SPEED_SCALE)) * -util.deadband(m_driverController.getLeftYAxis());
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
        m_robotDrive)
            .andThen(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true)));
  }
  private void initAutonChooser(){
    _autonChooser.setDefaultOption("Four Ball", new FourBallAuton());
    _autonChooser.addOption("Five Ball", new FiveBallAuton());
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if(_autonChooser.getSelected().equals(new FourBallAuton())){
    m_robotDrive.resetOdometry(new Pose2d(Trajectories.FourBall_AcquireFirstCargo().getInitialPose().getTranslation(), Trajectories.FourBall_AcquireFirstCargo().getInitialState().holonomicRotation));
    } else{
      m_robotDrive.resetOdometry(new Pose2d(Trajectories.FiveBall_AcquireFirstCargo().getInitialPose().getTranslation(), Trajectories.FourBall_AcquireFirstCargo().getInitialState().holonomicRotation));
    }
    return _autonChooser.getSelected().deadlineWith(new AutonTimer());
  }

}
