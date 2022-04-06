// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auton.AutonTimer;
import frc.robot.commands.auton.FiveBallAuton;
import frc.robot.commands.auton.FiveBallBackupAuton;
import frc.robot.commands.auton.FourBallAuton;
import frc.robot.commands.auton.FourBallBackupAuton;
import frc.robot.commands.auton.TwoBallBottomAuton;
import frc.robot.commands.auton.TwoBallMiddleAuton;
import frc.robot.commands.auton.TwoBallTopAuton;
import frc.robot.commands.auton.TwoBallTopGetOutOfTheWayAuton;
import frc.robot.commands.auton.TwoBallMiddleGetOutOfTheWayAuton;
import frc.robot.commands.chassis.RotateDrivetrainByLimelightAngle;
import frc.robot.commands.chassis.RotateDrivetrainToOdometryTargetAngle;
import frc.robot.commands.chassis.XDrive;
import frc.robot.commands.climber.HighBar;
import frc.robot.commands.climber.MidBar;
import frc.robot.commands.climber.MidToHigh;
import frc.robot.commands.climber.MoveArm;
import frc.robot.commands.conveyor.ReverseInfeedAndConveyor;
import frc.robot.commands.conveyor.RunConveyorOneBall;
import frc.robot.commands.conveyor.RunConveyorTwoBall;
import frc.robot.commands.BeakAutonCommand;
import frc.robot.commands.infeed.RunInfeedSingulatorMotors;
import frc.robot.commands.infeed.SetInfeedUp;
import frc.robot.commands.infeed.ToggleInfeedUp;
import frc.robot.commands.shooter.AcceptLimelightDistance;
import frc.robot.commands.shooter.DecrementShooterIndex;
import frc.robot.commands.shooter.IncrementShooterIndex;
import frc.robot.commands.shooter.ResetDefaultIndex;
import frc.robot.commands.shooter.RunShooterMotors;
import frc.robot.commands.vision.ToggleCamera;
import frc.robot.commands.vision.ToggleLEDMode;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;

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
  private final RunShooterMotors runShooter;
  private SlewRateLimiter driveXLimiter;
  private SlewRateLimiter driveYLimiter;
  private static RobotContainer _instance;
  private SendableChooser<BeakAutonCommand> _autonChooser = new SendableChooser<BeakAutonCommand>();

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
    runShooter = new RunShooterMotors();
    driveXLimiter = new SlewRateLimiter(6.);
    driveYLimiter = new SlewRateLimiter(6.);
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
    m_operatorController.x.toggleWhenPressed(runShooter);
    m_operatorController.y.toggleWhenPressed(runInfeed);
    m_operatorController.start.whenPressed(new SetInfeedUp());
    m_operatorController.back.whenPressed(new AcceptLimelightDistance());
    m_operatorController.lb.whenPressed(new DecrementShooterIndex(false));
    m_operatorController.rb.whenPressed(new IncrementShooterIndex(false));
    m_operatorController.lt.whenActive(new DecrementShooterIndex(true));
    m_operatorController.rt.whenActive(new IncrementShooterIndex(true));
    m_operatorController.ls.whenPressed(new ResetDefaultIndex());
    m_operatorController.rs.whenPressed(new InstantCommand(() -> Shooter.getInstance().setShooterIndex(17)));
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
    m_driverController.b.toggleWhenPressed(new RotateDrivetrainToOdometryTargetAngle());
    // ===================================

    // ======== TEMP. CLIMBER CONTROLLER
    BeakXBoxController climberController = new BeakXBoxController(2);
    Climber climber = Climber.getInstance();
    climberController.rb.whileHeld(new InstantCommand(() -> climber.leftMotorForward(.25)))
      .whenReleased(new InstantCommand(() -> climber.leftMotorOff()));
    climberController.lt.whileHeld(new InstantCommand(() -> climber.rightMotorBackward(-.25)))
      .whenReleased(new InstantCommand(() -> climber.rightMotorOff()));

    climberController.y.whileHeld(new InstantCommand(() -> climber.leftMotorForward(.8)).alongWith(new InstantCommand(() -> climber.rightMotorForward(.8))))
      .whenReleased(new InstantCommand(() -> climber.stop()));
    climberController.x.whileHeld(new InstantCommand(() -> climber.leftMotorBackward(-.8)).alongWith(new InstantCommand(() -> climber.rightMotorBackward(-.8))))
      .whenReleased(new InstantCommand(() -> climber.stop()));
    
    climberController.b.whenPressed(new InstantCommand(() -> climber.toggleGrippySolenoid()));
    climberController.lb.whileHeld(new InstantCommand(() -> climber.rightMotorForward(.25)))
      .whenReleased(new InstantCommand(() -> climber.rightMotorOff()));
    climberController.rt.whileHeld(new InstantCommand(() -> climber.leftMotorBackward(-.25)))
      .whenReleased(new InstantCommand(() -> climber.leftMotorOff()));

    // THIS IS ALL WORKING, DON'T CHANGE ANY OF THE COMMANDS
    climberController.back.whenPressed(new MoveArm(0.9, 153));
    climberController.back.whenPressed(new InstantCommand(() -> Infeed.getInstance().setInfeedDown()));
    climberController.back.whenPressed(new InstantCommand(() -> runShooter.cancel()));
    climberController.ls.whenPressed(new MidToHigh());
    climberController.start.whenPressed(new MidBar());

    climberController.rs.whenPressed(new HighBar());
    climberController.a.whenPressed(new InstantCommand(() -> climber.resetEncoders()));

    // ======= BRUH PIT CONTROLLER
    // BeakXBoxController pitController = new BeakXBoxController(3);

    // pitController.a.whenPressed(new InstantCommand(() -> climber.resetEncoders()));
    // pitController.start.whenPressed(new MoveArm(.2, 0));
    // pitController.back.whenPressed(new MoveArm(-.2, -35));

    // pitController.lb.whenPressed(new InstantCommand(() -> climber.setLeftEncoder(EncoderConstants.kClimberLeftStart)));
    // pitController.rb.whenPressed(new InstantCommand(() -> climber.setRightEncoder(EncoderConstants.kClimberRightStart)));

    // pitController.lt.whenPressed(new InstantCommand(() -> climber.slowDrop()));
    // pitController.rt.whenPressed(new InstantCommand(() -> climber.slowUp()));
  }

  public double getRightTrigger() {
    return m_driverController.getRightTrigger();
  }
  public double getSpeedScaledDriverRightX(){
    return util.speedscaleDrive(-util.deadband(m_driverController.getRightXAxis()), DriveConstants.BASE_SPEED_SCALE, getRightTrigger());
  }
  public double getSpeedScaledDriverLeftX(){
    return driveXLimiter.calculate(util.speedscaleDrive(-util.deadband(m_driverController.getLeftXAxis()), DriveConstants.BASE_SPEED_SCALE, getRightTrigger()));
  }
  public double getSpeedScaledDriverLeftY(){
    return driveYLimiter.calculate(util.speedscaleDrive(-util.deadband(m_driverController.getLeftYAxis()), DriveConstants.BASE_SPEED_SCALE, getRightTrigger()));
  }

  private void initAutonChooser(){
    _autonChooser.setDefaultOption("Four Ball", new FourBallAuton());
    _autonChooser.addOption("Four Ball With Backup", new FourBallBackupAuton());
    _autonChooser.addOption("Five Ball", new FiveBallAuton());
    _autonChooser.addOption("Five Ball With Backup", new FiveBallBackupAuton());
    _autonChooser.addOption("Top Two Ball", new TwoBallTopAuton());
    _autonChooser.addOption("Top Two Ball (Get Out Of The Way)", new TwoBallTopGetOutOfTheWayAuton());
    _autonChooser.addOption("Middle Two Ball (Get Out Of The Way)", new TwoBallMiddleGetOutOfTheWayAuton());
    _autonChooser.addOption("Middle Two Ball", new TwoBallMiddleAuton());    
    _autonChooser.addOption("Bottom Two Ball", new TwoBallBottomAuton());
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_robotDrive.resetOdometry(_autonChooser.getSelected().getInitialPose());
    return _autonChooser.getSelected().deadlineWith(new AutonTimer());
  }

}
