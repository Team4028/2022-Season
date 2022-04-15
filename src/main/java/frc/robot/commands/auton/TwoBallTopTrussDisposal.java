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
import frc.robot.commands.conveyor.SlowOutfeedAndConveyor;
import frc.robot.commands.shooter.ResetDefaultIndex;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Trajectories;

public class TwoBallTopTrussDisposal extends BeakAutonCommand{
  /** Creates a new TwoBallTopTrussDisposal. */
  public TwoBallTopTrussDisposal(){
  super.addCommands(
    new ResetDefaultIndex(),
    new InstantCommand(() -> Infeed.getInstance().setInfeedDown()),
    new WaitCommand(0.25),
    new InstantCommand(() -> Infeed.getInstance().forceRunInfeed()),
    util.getPathPlannerSwerveControllerCommand(Trajectories.TwoBall_Top()),
    new InstantCommand(() -> Shooter.getInstance().setShooterIndex(12.5, true)),
    new RotateDrivetrainToAngle(Rotation2d.fromDegrees(-32.0)),
    new InstantCommand(() -> Shooter.getInstance().runShooterMotors()),
    new WaitCommand(0.5),
    new RunConveyor().withTimeout(1.5),
    new InstantCommand(() -> Shooter.getInstance().stop()),
    new InstantCommand(() -> Infeed.getInstance().forceRunInfeed()),
    util.getPathPlannerSwerveControllerCommand(Trajectories.TwoBall_TopHangarDisposalFirstOpponentBall()),
    new WaitCommand(0.25),
    util.getPathPlannerSwerveControllerCommand(Trajectories.TwoBall_TopTrussDisposalSecond()),
    new WaitCommand(0.25),
    new InstantCommand(() -> Infeed.getInstance().stopInfeedSingulatorMotors()),
    util.getPathPlannerSwerveControllerCommand(Trajectories.TwoBall_TopTrussDispose()),
    new SlowOutfeedAndConveyor().withTimeout(1.0)//,
    //new InstantCommand(() -> Infeed.getInstance().setInfeedUp())
  );
  super.setInitialPose(Trajectories.TwoBall_Top());
}
}
