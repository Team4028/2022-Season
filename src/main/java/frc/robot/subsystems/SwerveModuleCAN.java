// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MK4IModuleConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModuleCAN {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  //private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;
  //private final AnalogInput m_offsetEncoder;

  private double turningMotorOffset;

  private final PIDController m_drivePIDController =
      new PIDController(0.0, 0, 0);

  private final PIDController m_turningPIDController = new PIDController(0.5, 0, 0.0);

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
    m_turningEncoder.configFactoryDefault();
    m_driveMotor.configFactoryDefault();
    m_turningMotor.configFactoryDefault();
    this.turningMotorOffset = turningMotorOffset;
    m_turningEncoder.setPositionToAbsolute();
    m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    m_turningMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    m_turningMotor.setSelectedSensorPosition((150/7) * 2048 * getTurningEncoderRadians() / (2 * Math.PI));

    //this.m_driveEncoder = new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setInverted(true);
    

    configMotorPID(m_turningMotor, 0, .2, 0.0, 0.1);
  

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private double getTurningEncoderRadians(){
    double angle = Math.toRadians(m_turningEncoder.getAbsolutePosition()) + turningMotorOffset;
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
        state.speedMetersPerSecond / DriveConstants.i_kMaxSpeedMetersPerSecond;
    // Calculate the turning motor output from the turning PID controller
    // Calculate the turning motor output from the turning PID controller.
    //m_turningMotor.setSelectedSensorPosition((150/7) * 2048 * getTurningEncoderRadians() / (2 * Math.PI));

    double desiredAngle = state.angle.getRadians();
    double currentAngle = getTurningEncoderRadians();
    double currentPulses = m_turningMotor.getSelectedSensorPosition();

    double deltaRadians = desiredAngle - currentAngle;
    double deltaPulses = (deltaRadians / (2 * Math.PI)) * MK4IModuleConstants.i_kEncoderCountsPerModuleRev;



    m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_turningMotor.set(ControlMode.Position, currentPulses + deltaPulses);
  }

  public void configMotorPID(TalonFX talon, int slotIdx, double p, double i, double d){
    talon.config_kP(slotIdx, p);
    talon.config_kI(slotIdx, i);
    talon.config_kD(slotIdx, d);
    //talon.config_kF(slotIdx, 0.4 * 1023/8360);
    talon.configMotionAcceleration(MK4IModuleConstants.kModuleMaxAccelerationTurningPulsesPer100MsSquared);
    talon.configMotionCruiseVelocity(MK4IModuleConstants.kModuleMaxSpeedTurningPulsesPer100Ms);
  }

//Zeros all the SwerveModule encoders.
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    m_turningMotor.setSelectedSensorPosition(0);
  }

public double mod(double a, double b){
  var r = a % b;
  if (r < 0) {
      r += b;
  }
  return r;
}
public double minChange(double a, double b, double wrap){
  return halfMod(a - b, wrap);
}

/**
* @return a value in range `[-wrap / 2, wrap / 2)` where `mod(a, wrap) == mod(value, wrap)`
*/
public double halfMod(double a, double wrap) {
  double aa = mod(a, wrap);
  double halfWrap = wrap / 2.0;
  if(aa >= halfWrap){
      aa -= wrap;
  }
  return aa;
}
}
