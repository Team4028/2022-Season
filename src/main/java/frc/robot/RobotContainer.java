// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BeakAutonCommand;
import frc.robot.commands.auton.AutonTimer;
import frc.robot.commands.auton.FiveBallAuton;
import frc.robot.commands.auton.FiveBallBackupAuton;
import frc.robot.commands.auton.FourBallAuton;
import frc.robot.commands.auton.FourBallBackupAuton;
import frc.robot.commands.auton.TwoBallBottomAuton;
import frc.robot.commands.auton.TwoBallMiddleAuton;
import frc.robot.commands.auton.TwoBallMiddleGetOutOfTheWayAuton;
import frc.robot.commands.auton.TwoBallTopAuton;
import frc.robot.commands.auton.TwoBallTopGetOutOfTheWayAuton;
import frc.robot.commands.auton.TwoBallTopHangarDisposal;
import frc.robot.commands.auton.TwoBallTopTrussDisposal;
import frc.robot.commands.chassis.RotateDrivetrainByLimelightAngle;
import frc.robot.commands.climber.HighBar;
import frc.robot.commands.climber.LeftZeroSequence;
import frc.robot.commands.climber.MidBar;
import frc.robot.commands.climber.MidToHigh;
import frc.robot.commands.climber.MoveArm;
import frc.robot.commands.climber.RightZeroSequence;
import frc.robot.commands.conveyor.ReverseInfeedAndConveyor;
import frc.robot.commands.conveyor.RunConveyorTwoBall;
import frc.robot.commands.infeed.EvacuateWrongCargo;
import frc.robot.commands.infeed.ReverseInfeedAndSingulator;
import frc.robot.commands.infeed.RunInfeedSingulatorMotors;
import frc.robot.commands.infeed.SetInfeedUp;
import frc.robot.commands.infeed.ToggleInfeedUp;
import frc.robot.commands.shooter.AcceptLimelightDistance;
import frc.robot.commands.shooter.DecrementShooterIndex;
import frc.robot.commands.shooter.IncrementShooterIndex;
import frc.robot.commands.shooter.MagicShootCommand;
import frc.robot.commands.shooter.RunShooterMotors;
import frc.robot.commands.shooter.RunShootersManual;
import frc.robot.commands.shooter.SetLongShot;
import frc.robot.commands.shooter.SetShortShot;
import frc.robot.commands.vision.ToggleCamera;
import frc.robot.commands.vision.ToggleLEDMode;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Infeed m_infeed = Infeed.getInstance();
  private final Limelight m_limelight = Limelight.getInstance();
  private final Conveyor m_conveyor = Conveyor.getInstance();
  private final Climber m_climber = Climber.getInstance();


  private final RunInfeedSingulatorMotors runInfeed;
  private final RunShooterMotors runShooter;
  private final SetLongShot setLongShotCommand;
  private final SetShortShot setShortShotCommand;

  private SlewRateLimiter driveXLimiter;
  private SlewRateLimiter driveYLimiter;
  private static RobotContainer _instance;
  private SendableChooser<BeakAutonCommand> _autonChooser = new SendableChooser<BeakAutonCommand>();
  private boolean fieldOriented = true;

  public static final RobotContainer getInstance() {
    if (_instance == null) {
      _instance = new RobotContainer();
    }
    return _instance;
  }

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
    setLongShotCommand = new SetLongShot();
    setShortShotCommand = new SetShortShot();

    driveXLimiter = new SlewRateLimiter(6.0);
    driveYLimiter = new SlewRateLimiter(6.0);
    configureButtonBindings();
    //Init Auton Chooser
    initAutonChooser();
    SmartDashboard.putData(_autonChooser);

    // Configure default commands
    m_drive.setDefaultCommand(
        new RunCommand(
            () -> m_drive.drive(
                getSpeedScaledDriverLeftY(),
                getSpeedScaledDriverLeftX(),
                getSpeedScaledDriverRightX(),
                () -> fieldOriented),
            m_drive
          )
      );
    m_shooter.setDefaultCommand(
      new RunCommand(
        () -> m_shooter.setShooterIndex(m_shooter.manualIndex()),
        m_shooter
        )
      );
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
    m_operatorController.y.whenPressed(new EvacuateWrongCargo().withTimeout(2.0));
    //m_operatorController.b.whenPressed(new RunConveyorOneBall());
    m_operatorController.b.whileActiveOnce(new RunShootersManual());
    m_operatorController.x.whileActiveOnce(new MagicShootCommand());
    m_operatorController.rs.toggleWhenPressed(new ReverseInfeedAndSingulator());
    m_operatorController.start.whenPressed(new SetInfeedUp());
    m_operatorController.back.whenPressed(new AcceptLimelightDistance());
    m_operatorController.lb.cancelWhenPressed(setShortShotCommand).cancelWhenPressed(setLongShotCommand).whenPressed(setLongShotCommand);
    m_operatorController.rb.cancelWhenPressed(setLongShotCommand).cancelWhenPressed(setShortShotCommand).whenPressed(setShortShotCommand);
    m_operatorController.lt.whenActive(new DecrementShooterIndex(true));
    m_operatorController.rt.whenActive(new IncrementShooterIndex(true));
    m_operatorController.ls.whenPressed(new InstantCommand(() -> m_shooter.toggleIsShotValidation()));
    // ====================================

    // ======== DRIVER CONTROLLER ========
    m_driverController.a.whenPressed(new ToggleLEDMode());
    m_driverController.x.whenPressed(new InstantCommand(() -> toggleFieldOriented()));
    m_driverController.start.whenPressed(new InstantCommand(() -> m_drive.zeroHeading()));
    m_driverController.lb.toggleWhenPressed(new ReverseInfeedAndConveyor());
    m_driverController.rb.whenPressed(new ToggleInfeedUp());
    m_driverController.lt.whileActiveContinuous(runInfeed);
    m_driverController.rs.whileActiveContinuous(new RotateDrivetrainByLimelightAngle(true));
    m_driverController.ls.whenPressed(new ToggleCamera());
    // ===================================

    // ======== TEMP. m_climber CONTROLLER
    BeakXBoxController m_climberController = new BeakXBoxController(2);
    m_climberController.rb.whileHeld(new InstantCommand(() -> m_climber.leftMotorForward(.25)))
      .whenReleased(new InstantCommand(() -> m_climber.leftMotorOff()));
    m_climberController.lt.whileHeld(new InstantCommand(() -> m_climber.rightMotorBackward(-.25)))
      .whenReleased(new InstantCommand(() -> m_climber.rightMotorOff()));

    m_climberController.y.whileHeld(new InstantCommand(() -> m_climber.leftMotorForward(.8)).alongWith(new InstantCommand(() -> m_climber.rightMotorForward(.8))))
      .whenReleased(new InstantCommand(() -> m_climber.stop()));
    m_climberController.x.whileHeld(new InstantCommand(() -> m_climber.leftMotorBackward(-.8)).alongWith(new InstantCommand(() -> m_climber.rightMotorBackward(-.8))))
      .whenReleased(new InstantCommand(() -> m_climber.stop()));
    
    m_climberController.b.whenPressed(new InstantCommand(() -> m_climber.toggleGrippySolenoid()));
    m_climberController.lb.whileHeld(new InstantCommand(() -> m_climber.rightMotorForward(.25)))
      .whenReleased(new InstantCommand(() -> m_climber.rightMotorOff()));
    m_climberController.rt.whileHeld(new InstantCommand(() -> m_climber.leftMotorBackward(-.25)))
      .whenReleased(new InstantCommand(() -> m_climber.leftMotorOff()));

    // THIS IS ALL WORKING, DON'T CHANGE ANY OF THE COMMANDS
    m_climberController.back.whenPressed(new MoveArm(0.9, 127));
    m_climberController.back.whenPressed(new InstantCommand(() -> m_infeed.setInfeedDown()));
    m_climberController.back.whenPressed(new InstantCommand(() -> runShooter.cancel()));
    m_climberController.ls.whenPressed(new MidToHigh());
    m_climberController.start.whenPressed(new MidBar());

    m_climberController.rs.whenPressed(new HighBar());
    m_climberController.a.whenPressed(new RightZeroSequence());
    m_climberController.a.whenPressed(new LeftZeroSequence());

    // ======= BRUH PIT CONTROLLER
    // BeakXBoxController pitController = new BeakXBoxController(3);

    // pitController.a.whenPressed(new InstantCommand(() -> m_climber.resetEncoders()));
    // pitController.start.whenPressed(new MoveArm(.2, 0));
    // pitController.back.whenPressed(new MoveArm(-.2, -35));

    // pitController.lb.whenPressed(new InstantCommand(() -> m_climber.setLeftEncoder(EncoderConstants.km_climberLeftStart)));
    // pitController.rb.whenPressed(new InstantCommand(() -> m_climber.setRightEncoder(EncoderConstants.km_climberRightStart)));

    // pitController.lt.whenPressed(new InstantCommand(() -> m_climber.slowDrop()));
    // pitController.rt.whenPressed(new InstantCommand(() -> m_climber.slowUp()));
  }

  public double getRightTrigger() {
    return m_driverController.getRightTrigger();
  }
  public double getSpeedScaledDriverRightX(){
    return util.speedscaleDrive(-m_driverController.getRightXAxis(), DriveConstants.BASE_SPEED_SCALE, getRightTrigger());
  }
  public double getSpeedScaledDriverLeftX(){
    return driveXLimiter.calculate(util.speedscaleDrive(-m_driverController.getLeftXAxis(), DriveConstants.BASE_SPEED_SCALE, getRightTrigger()));
  }
  public double getSpeedScaledDriverLeftY(){
    return driveYLimiter.calculate(util.speedscaleDrive(-m_driverController.getLeftYAxis(), DriveConstants.BASE_SPEED_SCALE, getRightTrigger()));
  }
  public void toggleFieldOriented(){
    fieldOriented = !fieldOriented;
  }
  private void initAutonChooser(){
    // _autonChooser.setDefaultOption("Four Ball", new FourBallAuton());
    _autonChooser.setDefaultOption("Four Ball With Backup", new FourBallBackupAuton());
    // _autonChooser.addOption("Five Ball", new FiveBallAuton());
    _autonChooser.addOption("Five Ball With Backup", new FiveBallBackupAuton());
    _autonChooser.addOption("Top Two Ball", new TwoBallTopAuton());
    _autonChooser.addOption("Top Two Ball (Get Out Of The Way)", new TwoBallTopGetOutOfTheWayAuton());
    _autonChooser.addOption("Middle Two Ball (Get Out Of The Way)", new TwoBallMiddleGetOutOfTheWayAuton());
    _autonChooser.addOption("Middle Two Ball", new TwoBallMiddleAuton());    
    _autonChooser.addOption("Bottom Two Ball", new TwoBallBottomAuton());
    _autonChooser.addOption("Two Ball Top Hangar Disposal", new TwoBallTopHangarDisposal());
    _autonChooser.addOption("Two Ball Top Truss Disposal", new TwoBallTopTrussDisposal());
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutonTimer timer = new AutonTimer();
    m_drive.resetOdometry(_autonChooser.getSelected().getInitialPose());
    return _autonChooser.getSelected().deadlineWith(timer);
  }

}
