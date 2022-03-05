// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final AnalogInput m_turningEncoder;
  //private final AnalogInput m_offsetEncoder;

  private double turningMotorOffset;

  private final PIDController m_drivePIDController =
      new PIDController(0.005, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  // private final ProfiledPIDController m_turningPIDController =
  //     new ProfiledPIDController(
  //         ModuleConstants.kPModuleTurningController,
  //         0,
  //         0.0001,
  //         new TrapezoidProfile.Constraints(
  //             ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
  //             ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  private final PIDController m_turningPIDController = new PIDController(ModuleConstants.kPModuleTurningController, 0, 0.0001);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int analogEncoderPort,
      double turningMotorOffset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    this.turningMotorOffset = turningMotorOffset;

    //this.m_driveEncoder = new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);
  
    m_turningEncoder = new AnalogInput(analogEncoderPort);
    //m_turningEncoder = m_driveMotor.getEncoder();

    m_driveEncoder = m_driveMotor.getEncoder();
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderDistancePerPulse);

    //m_turningEncoder.setPosition(turningMotorOffset);


    // Set whether drive encoder should be reversed or not
    // m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.

    // Set whether turning encoder should be reversed or not
    // m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private double getTurningEncoderRadians(){
    double angle = (1.0 - m_turningEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI + turningMotorOffset;
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
        angle += 2.0 * Math.PI;
    }
    return angle;
    }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getTurningEncoderRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderRadians()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        state.speedMetersPerSecond;//ontroller.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(getTurningEncoderRadians(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  /*public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }*/
}
