// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ColorSensor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EvacuateOuterCargo extends SequentialCommandGroup {
    private boolean correctColor = false;

    /** Creates a new EvacuateOuterCargo. */
    public EvacuateOuterCargo() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new WaitCommand(0.05),
                new WaitCommand(1.5).deadlineWith(
                        new ReverseInfeedAndSingulator()));
    }

    @Override
    public void initialize() {
        super.initialize();
        correctColor = ColorSensor.getInstance().getOuterBallCorrect();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || correctColor;
    }
}
