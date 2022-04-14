// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.conveyor.RunConveyorOneBallEvacuate;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EvacuateInnerCargo extends SequentialCommandGroup {
  boolean correctColor = false;
  /** Creates a new EvacuateInnerCargo. */
  public EvacuateInnerCargo() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(0.1),
      new WaitCommand(1.5).deadlineWith(
        new RunConveyorOneBallEvacuate(),
        new InstantCommand(() -> Shooter.getInstance().runShooterOutfeed(), Shooter.getInstance())
      ),
      new InstantCommand(() -> Shooter.getInstance().stop(), Shooter.getInstance())
    );
  }
  @Override
  public void initialize(){
    super.initialize();
    correctColor = ColorSensor.getInstance().getInnerBallCorrect();
  }
  @Override
  public boolean isFinished(){
    return super.isFinished() || correctColor;
  }
}
