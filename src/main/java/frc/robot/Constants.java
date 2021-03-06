// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // TODO: Better organization of Constants class
    public static final class DriveConstants {

        public static final boolean MK4I = true;
        public static final double BASE_SPEED_SCALE = 0.25;

        public static final int i_kFrontLeftDriveMotorPort = 2;
        public static final int i_kRearLeftDriveMotorPort = 6;
        public static final int i_kFrontRightDriveMotorPort = 4;
        public static final int i_kRearRightDriveMotorPort = 8;

        public static final int i_kFrontLeftTurningMotorPort = 1;
        public static final int i_kRearLeftTurningMotorPort = 5;
        public static final int i_kFrontRightTurningMotorPort = 3;
        public static final int i_kRearRightTurningMotorPort = 7;

        public static final int i_kFrontLeftEncoderCan = 1;
        public static final int i_kRearLeftEncoderCan = 3;
        public static final int i_kFrontRightEncoderCan = 2;
        public static final int i_kRearRightEncoderCan = 4;

        public static final int pigeonCan = 1;

        public static final String kCANivoreName = "DriveSubsystem";

        public static final double kTrackWidth = Units.inchesToMeters(23.75 - 2);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(25.75 - 2);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;

        public static final double ksVolts = (0.19 + 0.225 + 0.214 + 0.2256) / 4.0;// 0.798;
        public static final double kvVoltSecondsPerMeter = (2.2565 + 2.2785 + 2.2754 + 2.291) / 4.0;// 2.35;
        public static final double kaVoltSecondsSquaredPerMeter = (0.277 + 0.31) / 2.0;// 0.30;

        public static final SimpleMotorFeedforward driveTrainFeedforward = new SimpleMotorFeedforward(
                ksVolts,
                kvVoltSecondsPerMeter,
                kaVoltSecondsSquaredPerMeter);

        public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(16.3);
        public static final double i_kMaxSpeedMetersPerSecond = Units.feetToMeters(16.3);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class MK4IModuleConstants {
        public static final double i_kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double i_kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

        public static final double i_integratedEncoderTicksPerModRev = 2048.0 * (150.0 / 7.0);

        public static final int i_kEncoderCPR = 4096;
        public static final double i_kWheelDiameterMeters = Units.inchesToMeters(3.95);// Units.inchesToMeters(4.0);
        public static final double i_kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (i_kWheelDiameterMeters * Math.PI) * (1.0 / (50.0 / 14.0) / (17.0 / 27.0) / (45.0 / 15.0)) / 2048.0;

        public static final double i_kDriveEncoderCountsPerWheelRev = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)
                * 2048.0;

        public static final double i_kPModuleTurningController = 0.2;

        public static final double i_kPModuleDriveController = 0.10;

        public static final double i_kEncoderCountsPerModuleRev = (150.0 / 7.0) * 2048.0;

        public static final double i_kNominalVoltage = 12.0;

        public static final double i_kTurningMotorAllowableClosedLoopError = 40.0;

        public static final double kModuleMaxSpeedTurningRadiansPerSecond = 16 * Math.PI;
        public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared = 256 * Math.PI;
        public static final double kModuleMaxSpeedTurningPulsesPer100Ms = kModuleMaxSpeedTurningRadiansPerSecond
                * i_kEncoderCountsPerModuleRev * 0.1;
        public static final double kModuleMaxAccelerationTurningPulsesPer100MsSquared = kModuleMaxAccelerationTurningRadiansPerSecondSquared
                * i_kEncoderCountsPerModuleRev * 0.01;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(16.3);
        public static final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(16.3);

        public static final double kPXController = 8.75;
        public static final double kPYController = kPXController;
        public static final double kPThetaController = 7.0 * 0.9;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Units.feetToMeters(16.3) /
                Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0);
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        public static final ProfiledPIDController AUTON_THETA_CONTROLLER = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

        public static final TrajectoryConfig AutonTrajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);
        public static final PIDController AUTON_X_CONTROLLER = new PIDController(AutoConstants.kPXController, 0, 0);
        public static final PIDController AUTON_Y_CONTROLLER = new PIDController(AutoConstants.kPYController, 0, 0);
    }

    public static final class SubsystemConstants {
        /*************** CAN IDS **************/

        // 0-8 will be alotted for swerve motors
        // 9-12 will be alotted and encoders

        public static final int INFEED_MOTOR_ID = 13;
        public static final int SINGULATOR_MOTOR_ID = 14;
        public static final int CONVEYOR_MOTOR_ID = 15;
        public static final int KICKER_MOTOR_ID = 16;
        public static final int SHOOTER_FRONT_MOTOR_ID = 17;
        public static final int SHOOTER_BACK_MOTOR_ID = 18;
        public static final int ANGLE_MOTOR_ID = 19;

        public static final int CLIMBER_LEFT_MOTOR_ID = 20;
        public static final int CLIMBER_RIGHT_MOTOR_ID = 21;

        public static final int TOF1_SENSOR_ID = 420;
        public static final int TOF2_SENSOR_ID = 69;

        public static final int INFEED_SOLENOID_ID = 1;
        public static final int GRIPPY_SOLENOID_ID = 2;
        public static final int TIPPY_SOLENOID_ID = 0;
    }

    public static final class VBusConstants {
        public static final double kConveyAll = 0.75;
        public static final double kConveyOne = 0.50;// 0.85; // op b
        public static final double kConveyTwo = 0.75; // op a

        public static final double kInfeed = 0.85; // op y
        public static final double kSingulator = 0.5; // op y

        public static final double kShooterFrontDefault = 0.35;
        public static final double kShooterBackDefault = 0.55;
        public static final double kShooterHoodAngleRotDefault = 24.5;

        public static final double kClimberSlow = 0.6;
        public static final double kClimberFast = 1;
    }

    public static final class EncoderConstants {
        public static final double kConveyOne = 20;
        public static final double kConveyTwo = 100;

        public static final double kAngleThreshold = 0.1;

        public static final double kClimberLeftStart = 35.;
        public static final double kClimberRightStart = 35.;
    }

    public static final class ShooterConstants {
        public static final double kFineAdjustment = 0.5;
        public static final double kCoarseAdjustment = 1.;

        public static final double kIndexDefault = 12.5;

        public static final double kMaxAllowedAngle = 33.;

        public static final boolean kIsVBus = true;
        public static final boolean kIsRealGoal = true;

        public static final boolean kUseVoltageComp = true;
        public static final double kVoltageCompensation = 10.75;
    }

    public static final class LimelightConstants {
        public static final double kTargetHeight = ShooterConstants.kIsRealGoal ? 104. : 85.;
        public static final double kMountHeight = 21;
        public static final double kMountAngle = 35.45;//34.5;//29.4;// 30.5;

        public static final double kLensToBack = 22. - 1.2;
        public static final double kTapeToCenter = 26.;
    }

    public static final class VisionConstants {
        public static final String kCamera1Url = "http://10.40.28.13:1182/stream.mjpg"; // infeed
        public static final String kCamera2Url = "http://10.40.28.13:1181/stream.mjpg"; // shooter
    }

    public static final class PIDConstants {
        // TODO: These are **only** tuned for the 12.5 foot auton shot.
        // So at greater distances it overshoots the second ball.
        // This can be fixed--and nothing can realistically be lost--
        // by upping the D a bit.
        public static final class Front {
            public static double kF = 0.047;
            public static double kP = 0.1;
            public static double kD = 0.12;
            public static double kMax = 22000; // 20400;
        }

        public static final class Back {
            public static double kF = 0.047;
            public static double kP = 0.1;
            public static double kD = 0.12;
            public static double kMax = 22000; // 17000;
        }

        public static final class Angle {
            public static double kP = 0.1;
        }
    }

    public static final class ControllerConstants {
        public static final double kDeadband = 0.025; // Jiggle room for the thumbsticks
        public static final double kSensitivity = 0.025;
        public static final double kTriggerDeadband = 0.01; // Jiggle room for the triggers
        public static final double kTriggerSensitivity = 0.6; // If the trigger is beyond this limit, say it has been
                                                              // pressed

        /* Button Mappings */
        public static final class Buttons {
            public static final int kA = 1;
            public static final int kB = 2;
            public static final int kX = 3;
            public static final int kY = 4;
            public static final int kLB = 5;
            public static final int kRB = 6;
            public static final int kBack = 7;
            public static final int kStart = 8;
            public static final int kLS = 9;
            public static final int kRS = 10;
        }

        /* Axis Mappings */
        public static final class Axes {
            public static final int kLeftX = 0;
            public static final int kLeftY = 1;
            public static final int kLeftTrigger = 2;
            public static final int kRightTrigger = 3;
            public static final int kRightX = 4;
            public static final int kRightY = 5;
        }
    }

    public static final class CurrentLimitConstants {
        public static final int kClimber = 60;
        public static final int kConveyor = 20;
        public static final int kAngle = 10;
    }

    public static final class RampRateConstants {
        public static final double kClimber = 0.1;
    }

    public static final class ColorSensorConstants {
        public static final class InfeedBall {
            public static final int kRedThreshold = 300;
            public static final int kBlueThreshold = 350;
            public static final int kProximityThreshold = 120;
        }

        public static final class ShooterBall {
            public static final int kRedThreshold = 300;
            public static final int kBlueThreshold = 400;
            public static final int kProximityThreshold = 180;
        }
    }
}
