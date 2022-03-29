// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BeakAutonCommand extends SequentialCommandGroup {
  /** Creates a new BeakAutonCommand. */
  private Pose2d initialPose;
  public BeakAutonCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
  protected void setInitialPose(PathPlannerTrajectory initialTrajectory){
    this.initialPose = new Pose2d(initialTrajectory.getInitialPose().getTranslation(), initialTrajectory.getInitialState().holonomicRotation);
  }
  public Pose2d getInitialPose(){
    return initialPose;
  }
}
