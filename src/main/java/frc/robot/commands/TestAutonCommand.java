// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutonCommand extends SequentialCommandGroup {
  /** Creates a new TestAutonCommand. */
  private Trajectories _trajectories = Trajectories.get_instance();
  private RobotContainer _rc = RobotContainer.get_instance();
  public TestAutonCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      _rc.getPathPlannerSwerveControllerCommand(_trajectories.FourBall_AcquireFirstCargo()).alongWith(new InstantCommand(() -> Infeed.get_instance().toggleInfeedRun())),
      new RotateDrivetrainByAngle(Rotation2d.fromDegrees(-165), true).alongWith(new InstantCommand(() -> Shooter.getInstance().runShooterMotorsVbus())),
      new WaitCommand(1.0).deadlineWith(new RunConveyor()),
      new InstantCommand(() -> Shooter.getInstance().stop()),
      _rc.getPathPlannerSwerveControllerCommand(_trajectories.FourBall_AcquireLoadingZoneCargo()),
      new WaitCommand(2.0),
      _rc.getPathPlannerSwerveControllerCommand(_trajectories.FourBall_ReturnToShoot()).alongWith(new WaitCommand(0.5).andThen(new InstantCommand(() -> Shooter.getInstance().runShooterMotorsVbus())),
      new WaitCommand(1.0).deadlineWith(new RunConveyor()),
      new InstantCommand(() -> Shooter.getInstance().stop()),
      new InstantCommand(() -> Infeed.get_instance().toggleInfeedRun()))
    );
  }
}
