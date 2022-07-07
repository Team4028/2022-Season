// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TestAll;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private Shooter shooter;
    private Limelight limelight;
    private Climber climber;
    private ColorSensor colorSensor;
    private DriveSubsystem drive;

    private CommandScheduler commandScheduler;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = RobotContainer.getInstance();
        LiveWindow.disableAllTelemetry();

        colorSensor = ColorSensor.getInstance();
        climber = Climber.getInstance();
        limelight = Limelight.getInstance();
        shooter = Shooter.getInstance();
        drive = DriveSubsystem.getInstance();
        commandScheduler = CommandScheduler.getInstance();
        
        limelight.setPipeline(ShooterConstants.kIsRealGoal ? 5 : 5);
        limelight.setPictureInPicture(0);
        limelight.setLedMode(1);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        commandScheduler.run();
        // SmartDashboard.putBoolean("Compressor", compressor.enabled());

    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        limelight.setLedMode(1.);
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        colorSensor.setAlliance();
        limelight.setLedMode(0.);
        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
         * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
         * = new MyAutoCommand(); break; case "Default Auto": default:
         * autonomousCommand = new ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        colorSensor.setAlliance();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        shooter.resetCounter();
        shooter.setIsShotValidation(false);
        climber.resetEncoders();
        limelight.setLedMode(0.);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if(limelight.getHasTarget()){
            drive.inputVisionPose(limelight.cameraToTarget(), Units.millisecondsToSeconds(limelight.getPipelineLatency() + 11));
        }
    }

    @Override
    public void testInit() {

        // Cancels all running commands at the start of test mode.
        // CommandScheduler.getInstance().cancelAll();

        // new TestAll().schedule();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
