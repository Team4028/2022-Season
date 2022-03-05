// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util;
import frc.robot.Constants.MK4IModuleConstants;

public class SwerveModuleCAN {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  //private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;
  //private final AnalogInput m_offsetEncoder;

  private double turningMotorOffset;

  private final PIDController m_drivePIDController =
      new PIDController(0.0, 0, 0);

  private final PIDController m_turningPIDController = new PIDController(MK4IModuleConstants.i_kPModuleTurningController, 0, 0.0);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModuleCAN(
      int driveMotorChannel,
      int turningMotorChannel,
      int CANEncoderPort,
      double turningMotorOffset) {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    m_turningEncoder = new CANCoder(CANEncoderPort);
    this.turningMotorOffset = turningMotorOffset;
    m_turningEncoder.setPositionToAbsolute();

    //this.m_driveEncoder = new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);
    m_driveMotor.configFactoryDefault();
    m_turningMotor.configFactoryDefault();
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setNeutralMode(NeutralMode.Coast);
  

    m_turningEncoder.configFeedbackCoefficient(2 * Math.PI / MK4IModuleConstants.i_kEncoderCPR, "rad", SensorTimeBase.PerSecond);
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    //m_turningEncoder = m_driveMotor.getEncoder();
    m_turningMotor.config_kP(0, MK4IModuleConstants.i_kPModuleTurningController);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private double getTurningEncoderRadians(){
    double angle = m_turningEncoder.getAbsolutePosition() + turningMotorOffset;
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
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(getTurningEncoderRadians()));
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
        state.speedMetersPerSecond/util.feetToMeters(12);//ontroller.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(getTurningEncoderRadians(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_turningMotor.set(ControlMode.Position, turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  /*public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }*/
}
