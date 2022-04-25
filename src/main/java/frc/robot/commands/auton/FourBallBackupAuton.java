// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util;
import frc.robot.commands.BeakAutonCommand;
import frc.robot.commands.chassis.RotateDrivetrainToAngle;
import frc.robot.commands.conveyor.RunConveyor;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallBackupAuton extends BeakAutonCommand {
    /** Creates a new FourBallBackupAuton. */
    public FourBallBackupAuton() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        super.setInitialPose(Trajectories.FourBall_AcquireFirstCargo());
        super.addCommands(
                new InstantCommand(() -> Shooter.getInstance().setShooterIndex(12.8, true), Shooter.getInstance()),
                new InstantCommand(() -> Infeed.getInstance().setInfeedDown()),
                new WaitCommand(0.25),
                new InstantCommand(() -> Infeed.getInstance().forceRunInfeed(), Infeed.getInstance()),
                util.getPathPlannerSwerveControllerCommand(Trajectories.FourBall_AcquireFirstCargo()),
                new WaitCommand(0.5),
                new RotateDrivetrainToAngle(Rotation2d.fromDegrees(37.0))
                        .alongWith(new InstantCommand(() -> Shooter.getInstance().runShooterMotors())),
                new WaitCommand(1.1).deadlineWith(new RunConveyor()),
                new InstantCommand(() -> Shooter.getInstance().stop()),
                new InstantCommand(() -> Infeed.getInstance().forceRunInfeed(), Infeed.getInstance()),
                util.getPathPlannerSwerveControllerCommand(Trajectories.FourBall_AcquireLoadingZoneCargo()),
                new WaitCommand(0.75),
                util.getPathPlannerSwerveControllerCommand(Trajectories.FourBall_Backup()),
                new WaitCommand(1.5),
                util.getPathPlannerSwerveControllerCommand(Trajectories.FourBall_BackupReturnToShoot()),
                new RotateDrivetrainToAngle(Trajectories.FourBall_BackupReturnToShoot().getEndState().holonomicRotation)
                        .withTimeout(1.5),
                new InstantCommand(() -> Shooter.getInstance().runShooterMotors()),
                new WaitCommand(0.25),
                new WaitCommand(1.1).deadlineWith(new RunConveyor()),
                new WaitCommand(0.25).andThen(new InstantCommand(() -> Shooter.getInstance().stop())),
                new InstantCommand(() -> Infeed.getInstance().stopInfeedSingulatorMotors()));
    }
}
