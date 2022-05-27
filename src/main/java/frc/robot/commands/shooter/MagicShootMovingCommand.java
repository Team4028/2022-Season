// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.chassis.RotateDrivetrainByLimelightAngle;
import frc.robot.commands.chassis.RotateDrivetrainToOdometryTargetAngle;
import frc.robot.commands.conveyor.RunConveyorTwoBall;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MagicShootMovingCommand extends SequentialCommandGroup {
    Shooter shooter;
    Limelight limelight;
    DriveSubsystem drive;

    AcceptLimelightDistance acceptDist;
    RotateDrivetrainByLimelightAngle continuousRotate;
    /** Creates a new MagicShootMovingCommand. */
    /**
     * false = before conveyor runs
     * true = after conveyor runs
     */
    private boolean interruptPoint;

    public MagicShootMovingCommand() {
        // Add your commands in the addCommands() call, e.g.
        interruptPoint = false;
        // addCommands(new FooCommand(), new BarCommand());
        addRequirements(Shooter.getInstance());
        addCommands(
                new RotateDrivetrainToOdometryTargetAngle(),
                new WaitUntilCommand(() -> limelight.getHasTarget()),
                new InstantCommand(() -> setInterruptPoint(false)),
                // new RotateDrivetrainByLimelightAngle(false),
                // new AcceptLimelightDistance(),
                sequence(
                        new WaitCommand(0.25),
                        new InstantCommand(() -> setInterruptPoint(true)),
                        new RunConveyorTwoBall(),
                        new InstantCommand(() -> Vision.getInstance().setInfeedCamera()),
                        new InstantCommand(() -> setInterruptPoint(false)),
                        new WaitCommand(0.25)).deadlineWith(
                                new RunShooterMotors()
                // new RotateDrivetrainByLimelightAngle(true)
                ));
    }

    @Override
    public boolean isFinished() {
        return (isOutOfRangeOfManual() || super.isFinished());// || !Limelight.getInstance().getHasTarget());
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || isOutOfRangeOfManual()) { // || !Limelight.getInstance().getHasTarget()) {
            if (interruptPoint == false) {
                Shooter.getInstance().stop();
                Vision.getInstance().setInfeedCamera();
                Conveyor.getInstance().setIsRunning(false);
            } else {
                Vision.getInstance().setInfeedCamera();
                Conveyor.getInstance().setIsRunning(true);
                new MagicShootEndCommand().schedule();
            }
        } else {
            super.end(interrupted);
        }
    }

    @Override
    public void execute() {
        super.execute();

        acceptDist.schedule();
        continuousRotate.schedule();
    }

    private void setInterruptPoint(boolean pt) {
        interruptPoint = pt;
    }

    private boolean isOutOfRangeOfManual() {
        return Shooter.getInstance().getIsShotValidation() &&
                (!interruptPoint) &&
                Math.abs(Shooter.getInstance().manualIndex() - Shooter.getInstance().index()) > 3.0;
    }
}
