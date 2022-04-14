// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.infeed;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ColorSensor;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EvacuateThirdCargo extends ParallelRaceGroup {
  boolean bothCorrect = false;
  boolean bothFilled = false;
  /** Creates a new EvacuateThirdCargo. */
  public EvacuateThirdCargo() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(1.0),
      new ReverseInfeed()
    );
  }
  @Override
  public void initialize(){
    super.initialize();
    bothCorrect = ColorSensor.getInstance().getInnerBallCorrect() && ColorSensor.getInstance().getOuterBallCorrect();
    bothFilled = ColorSensor.getInstance().getInnerFilled() && ColorSensor.getInstance().getOuterFilled();
  }
  @Override
  public boolean isFinished(){
    return super.isFinished() || !bothCorrect || !bothFilled;
  }
}
