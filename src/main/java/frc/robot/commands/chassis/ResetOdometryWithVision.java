// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class ResetOdometryWithVision extends CommandBase {
    DriveSubsystem drive;
    Limelight limelight;

    /** Creates a new ResetOdometryWithVision. */
    public ResetOdometryWithVision() {
        drive = DriveSubsystem.getInstance();
        limelight = Limelight.getInstance();
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(drive);
        // addRequirements(limelight);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.resetOdometryWithVision(
            Units.feetToMeters(limelight.willTestDistance() - 1.)
        );
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
