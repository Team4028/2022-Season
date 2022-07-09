// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.chassis.ResetOdometryWithVision;
import frc.robot.commands.chassis.RotateDrivetrainByLimelightAngle;
import frc.robot.commands.chassis.RotateDrivetrainToAngleContinuous;
import frc.robot.commands.conveyor.RunConveyorOneBall;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MagicShootMovingCommand extends SequentialCommandGroup {
    Shooter shooter;
    Limelight limelight;
    DriveSubsystem drive;

    /** Creates a new MagicShootMovingCommand. */
    /**
     * false = before conveyor runs
     * true = after conveyor runs
     */
    private boolean interruptPoint;

    /** Whether or not to run periodics (rotate and set shooter index) */
    private boolean runPeriodics;

    /** Constant for the location of the goal */
    private Translation2d target;

    /** takes ~0.5 sec for the ball to exit */
    private double shotTime = 1.35;

    /** locked-controller speeds */
    private double leftXSpeed = 0.;
    private double leftYSpeed = 0.;

    public MagicShootMovingCommand() {
        // init target
        target = new Translation2d(
                Units.inchesToMeters(324.),
                Units.inchesToMeters(162.));

        // define subsystems
        shooter = Shooter.getInstance();
        drive = DriveSubsystem.getInstance();
        limelight = Limelight.getInstance();

        // define commands & vars
        interruptPoint = false;
        runPeriodics = false;

        addRequirements(Shooter.getInstance());
        addCommands(
                // new RotateDrivetrainToOdometryTargetAngle(),

                // sequence(
                // new WaitCommand(0.25),
                // new InstantCommand(() -> setInterruptPoint(true)),
                // new RunConveyorTwoBall(),
                // new InstantCommand(() -> Vision.getInstance().setInfeedCamera()),
                // new InstantCommand(() -> setInterruptPoint(false)),
                // new InstantCommand(() -> runPeriodics = false),
                // new WaitCommand(0.25)).deadlineWith(
                // new RunShooterMotors()
                // )
                new InstantCommand(() -> setInterruptPoint(false)),
                new ConditionalCommand(
                        sequence(
                                new InstantCommand(() -> shooter.setIsVbus(false)),
                                new WaitUntilCommand(() -> limelight.getHasTarget()).withTimeout(0.1),
                                new ConditionalCommand(
                                        new InstantCommand(() -> {
                                        }),
                                        new InstantCommand(() -> this.cancel()),
                                        () -> limelight.getHasTarget()),
                                new WaitUntilCommand(() -> limelight.getX() < 3.5)
                                        .deadlineWith(new RotateDrivetrainByLimelightAngle(true)),
                                new AcceptLimelightDistance(),
                                new InstantCommand(() -> setInterruptPoint(true)),
                                new ResetOdometryWithVision(),
                                new InstantCommand(() -> runPeriodics = true),
                                new InstantCommand(() -> resetControllerSpeeds()),
                                new WaitCommand(0.55).deadlineWith(
                                        new RotateDrivetrainToAngleContinuous(() -> getAngleToMovingGoal(),
                                                () -> leftXSpeed,
                                                () -> leftYSpeed)),
                                parallel(
                                        sequence(
                                                new InstantCommand(
                                                        () -> shooter.setShooterIndex(getDistanceToMovingGoal() + 2.5,
                                                                false)),
                                                new WaitCommand(0.4),
                                                new RunConveyorOneBall(),
                                                new InstantCommand(() -> setInterruptPoint(false)),
                                                new WaitCommand(0.3),
                                                new InstantCommand(() -> shooter.setIsVbus(true))
                                                ),
                                        new InstantCommand(() -> drive.drive(
                                                leftYSpeed,
                                                leftXSpeed,
                                                0,
                                                () -> true))))
                                                        .deadlineWith(
                                                                new RunShooterMotors()),

                        new MagicShootCommand(),
                        () -> Math.hypot(
                                drive.getDriveXVelocity(),
                                drive.getDriveYVelocity()) > 0.18));
    }

    private void resetControllerSpeeds() {
        leftXSpeed = RobotContainer.getInstance().getSpeedScaledDriverLeftX();
        leftYSpeed = RobotContainer.getInstance().getSpeedScaledDriverLeftY();
    }

    private Translation2d getMovingGoal() {
        ChassisSpeeds vels = drive.getFieldRelativeChassisSpeeds();

        double distance = limelight.willTestDistance();

        if (distance > 10.) {
            shotTime += .095 * (distance - 9.);
        }

        // Treat the robot as stationary and the goal as moving.
        // This "predicts" the location the goal will be in when the ball is shot
        double movingGoalX = target.getX() - shotTime * vels.vxMetersPerSecond;
        double movingGoalY = target.getY() - shotTime * vels.vyMetersPerSecond;

        Translation2d movingGoalLocation = new Translation2d(movingGoalX, movingGoalY);

        Translation2d toMovingGoal = movingGoalLocation.minus(drive.getPose().getTranslation());

        if (distance > 10.) {
            shotTime -= .095 * (distance - 9.);
        }
        
        return toMovingGoal;
    }

    /** In meters */
    private double getDistanceToMovingGoal() {
        Translation2d movingGoal = getMovingGoal();

        double dist = Units.metersToFeet(movingGoal.getDistance(new Translation2d()));

        return dist;
    }

    // In radians
    private Rotation2d getAngleToMovingGoal() {
        Translation2d movingGoal = getMovingGoal();

        Rotation2d angle = new Rotation2d(
                Math.atan2(
                        movingGoal.getY(),
                        movingGoal.getX()));

        return angle;
    }

    @Override
    public boolean isFinished() {
        return (isOutOfRangeOfManual() || super.isFinished());// || !Limelight.getInstance().getHasTarget());
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: Implement proper interrupt-point logic
        drive.drive(0, 0, 0, () -> true);
        if (interrupted || isOutOfRangeOfManual()) { // || !Limelight.getInstance().getHasTarget()) {
            if (interruptPoint == false) {
                shooter.stop();
                Vision.getInstance().setInfeedCamera();
                Conveyor.getInstance().setIsRunning(false);
            } else {
                Vision.getInstance().setInfeedCamera();
                Conveyor.getInstance().setIsRunning(true);
                new MagicShootEndCommand().schedule();
            }
        } else {
            super.end(interrupted);
        }
    }

    @Override
    public void execute() {
        super.execute();

        if (runPeriodics) {
            // shooter.setShooterIndex(getDistanceToMovingGoal() + 1., false);
            // continuousRotate.schedule();
        }
    }

    private void setInterruptPoint(boolean pt) {
        interruptPoint = pt;
    }

    private boolean isOutOfRangeOfManual() {
        return shooter.getIsShotValidation() &&
                (!interruptPoint) &&
                Math.abs(shooter.manualIndex() - shooter.index()) > 3.0;
    }
}
