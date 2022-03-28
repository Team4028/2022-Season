// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util;
import frc.robot.commands.conveyor.RunConveyor;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuton extends SequentialCommandGroup {
  /** Creates a new FiveBallAuotn. */
  public FiveBallAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> Infeed.getInstance().setInfeedDown()),
      new InstantCommand(() -> Infeed.getInstance().runInfeedSingulatorMotors(1.0)),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireFirstCargo())
      .alongWith(new WaitCommand(0.75).andThen(new InstantCommand(() -> Shooter.getInstance().runShooterMotors()))),
      new WaitCommand(1.1).deadlineWith(new RunConveyor()),
      new InstantCommand(() -> Shooter.getInstance().stop()),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireSecondCargo())
      .alongWith(new WaitCommand(1.5).deadlineWith(new RunConveyor())),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireLoadingZoneCargo()),
      new WaitCommand(1.2),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_ReturnToShoot())
      .alongWith(new WaitCommand(1.5).andThen(new InstantCommand(() -> Shooter.getInstance().runShooterMotors()))),
      new WaitCommand(1.2).deadlineWith(new RunConveyor()),
      new WaitCommand(0.25),
      new InstantCommand(() -> Shooter.getInstance().stop())
    );
  }
}
