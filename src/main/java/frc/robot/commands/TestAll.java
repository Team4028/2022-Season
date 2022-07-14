// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.climber.MoveArm;
import frc.robot.commands.climber.ToggleGrippy;
import frc.robot.commands.climber.ToggleTippy;
import frc.robot.commands.conveyor.RunConveyorTwoBall;
import frc.robot.commands.infeed.RunInfeedSingulatorMotors;
import frc.robot.commands.shooter.RunShooterMotors;
import frc.robot.commands.shooter.SetLongShot;
import frc.robot.commands.shooter.SetShortShot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAll extends SequentialCommandGroup {
    private Climber climber = Climber.getInstance();
    // private Conveyor conveyor = Conveyor.getInstance();
    private Infeed infeed = Infeed.getInstance();
    private Limelight limelight = Limelight.getInstance();
    // private Shooter shooter = Shooter.getInstance();

    private RunShooterMotors runShooter = new RunShooterMotors();
    private SetShortShot sh = new SetShortShot();
    private SetLongShot lo = new SetLongShot();

    private WaitCommand wait = new WaitCommand(0.5);

    private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    /** Creates a new TestAll. */
    public TestAll() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new InstantCommand(() -> limelight.setLedMode(1)),
            new InstantCommand(() -> climber.setCoast()),
            new WaitCommand(5.0), // 5 secs to pull the rope
            new InstantCommand(() -> climber.setBrake()),
            wait,
            new InstantCommand(() -> climber.resetEncoders()),
            new WaitUntilCommand(() -> !compressor.enabled()),
            new InstantCommand(() -> infeed.setInfeedDown()),
            wait,
            new WaitCommand(2.0).deadlineWith(new RunInfeedSingulatorMotors()),
            new RunConveyorTwoBall(),
            parallel(
                runShooter,
                sequence(
                    wait,
                    sh,
                    wait,
                    sh,
                    wait,
                    sh,
                    wait,
                    lo,
                    new WaitCommand(1.0),
                    lo,
                    wait,
                    lo,
                    wait
                )
            ),
            new InstantCommand(() -> runShooter.cancel()),
            new MoveArm(0.8, 130, false),
            wait,
            new ToggleGrippy(),
            new WaitCommand(2.0),
            new ToggleGrippy(),
            new WaitCommand(1.0),
            new ToggleTippy(),
            wait,
            new ToggleTippy(),
            wait,
            new MoveArm(-0.8, -130, false)
        );
    }
}