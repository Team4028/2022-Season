// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Infeed;

public class RunInfeedSingulatorMotors extends CommandBase {
    /** Creates a new ToggleOneAndTwo. */
    private Infeed infeed = Infeed.getInstance();

    public RunInfeedSingulatorMotors() {
        addRequirements(infeed);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        infeed.runInfeedSingulatorMotors(1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        infeed.runInfeedSingulatorMotors(1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        infeed.stopInfeedSingulatorMotors();
        SmartDashboard.putBoolean("Infeed/Running", false);
        SmartDashboard.putBoolean("Singulator/Running", false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
