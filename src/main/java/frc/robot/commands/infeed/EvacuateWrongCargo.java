// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;

public class EvacuateWrongCargo extends CommandBase {
  /** Creates a new EvacuateWrongCargo. */
  EvacuateThirdCargo ethird;
  EvacuateInnerCargo einner;
  EvacuateOuterCargo eouter;
  public EvacuateWrongCargo() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
    ethird = new EvacuateThirdCargo();
    einner = new EvacuateInnerCargo();
    eouter = new EvacuateOuterCargo();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(ColorSensor.getInstance().getInnerBallCorrect() && ColorSensor.getInstance().getOuterBallCorrect()){
      ethird.schedule();
    } else if(!ColorSensor.getInstance().getInnerBallCorrect() && !ColorSensor.getInstance().getOuterBallCorrect() && ColorSensor.getInstance().getInnerFilled() && ColorSensor.getInstance().getOuterFilled()){
      eouter.schedule();
      einner.schedule();
    } else if(ColorSensor.getInstance().getInnerBallCorrect() && !ColorSensor.getInstance().getOuterBallCorrect() && ColorSensor.getInstance().getOuterFilled()){
      eouter.schedule();
    } else if(!ColorSensor.getInstance().getInnerBallCorrect() && ColorSensor.getInstance().getOuterBallCorrect() && ColorSensor.getInstance().getInnerFilled()){
      einner.schedule();
    }
    System.out.println("Initialize Evacuate Wrong Cargo");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ethird.cancel();
    einner.cancel();
    eouter.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ethird.isFinished() && einner.isFinished() && eouter.isFinished();
  }
}
