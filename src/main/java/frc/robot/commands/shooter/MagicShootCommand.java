// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.chassis.RotateDrivetrainByLimelightAngle;
import frc.robot.commands.conveyor.RunConveyorTwoBall;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MagicShootCommand extends SequentialCommandGroup {
  /** Creates a new MagicShootCommand. */
  RunShooterMotors _RunShooterMotors;
  /**
   * false = before conveyor runs
   * true = after conveyor runs
   */
  private boolean interruptPoint;
  public MagicShootCommand() {
    // Add your commands in the addCommands() call, e.g.
    _RunShooterMotors = new RunShooterMotors();
    interruptPoint = false;
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> setInterruptPoint(false)),
      new RotateDrivetrainByLimelightAngle(false),
      new AcceptLimelightDistance(),
        sequence(
          new WaitCommand(0.25),
          new InstantCommand(() -> setInterruptPoint(true)),
          new RunConveyorTwoBall(),
          new WaitCommand(0.25)
        ).deadlineWith(
          new RunShooterMotors(),
          new RotateDrivetrainByLimelightAngle(true)
        )
    );
  }
  @Override
  public boolean isFinished(){
    return super.isFinished() || !Limelight.getInstance().getHasTarget();
  }
  @Override
  public void end(boolean interrupted){
    if(interrupted){
      if(interruptPoint == false){
        Shooter.getInstance().stop();
        Vision.getInstance().setInfeedCamera();
        Conveyor.getInstance().setIsRunning(false);
      } else{
        Conveyor.getInstance().setIsRunning(true);
      }
    } else{
      super.end(interrupted);
    }
    new MagicShootEndCommand().schedule();
  }
  private void setInterruptPoint(boolean pt){
    interruptPoint = pt;
  }
}
