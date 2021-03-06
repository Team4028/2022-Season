// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AcceptLimelightDistance extends CommandBase {
    private Shooter _s = Shooter.getInstance();
    private Limelight _l = Limelight.getInstance();

    /** Creates a new AcceptLimelightDistance. */
    public AcceptLimelightDistance() {
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(_s);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _s.setShooterIndex(_l.willTestDistance(), false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
