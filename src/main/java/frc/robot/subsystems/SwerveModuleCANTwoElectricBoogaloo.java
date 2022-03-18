// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.MK4IModuleConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class SwerveModuleCANTwoElectricBoogaloo {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;
  private final WPI_CANCoder m_turningEncoder;

  private final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private final int CAN_TIMEOUT_MS = 250;
  int dub;

  private int resetIterations = 0;

  /**
   * Constructs a SwerveModuleCANTwoElectricBoogaloo.
   * Current best solution for MK4i Chassis
   * 
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModuleCANTwoElectricBoogaloo(
      int driveMotorChannel,
      int turningMotorChannel,
      int CANEncoderPort,
      double turningMotorOffset) {
    m_driveMotor = new WPI_TalonFX(driveMotorChannel, DriveConstants.kCANivoreName);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel, DriveConstants.kCANivoreName);
    m_turningEncoder = new WPI_CANCoder(CANEncoderPort, DriveConstants.kCANivoreName);
    dub = CANEncoderPort;

    // m_turningEncoder.setPositionToAbsolute();
    // if(m_turningEncoder.configGetSensorInitializationStrategy() !=
    // SensorInitializationStrategy.BootToAbsolutePosition){
    // m_turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition,
    // CAN_TIMEOUT_MS);
    // m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    // }
    // m_turningEncoder.configFactoryDefault();
    // m_turningMotor.configFactoryDefault();
    // m_driveMotor.configFactoryDefault();

    m_turningEncoder.configMagnetOffset(Math.toDegrees(turningMotorOffset));
    // m_turningMotor.setSensorPhase(false);
    // m_turningMotor.configAllowableClosedloopError(0,
    // i_kTurningMotorAllowableClosedLoopError, CAN_TIMEOUT_MS);
    // m_turningMotor.setStatusFramePeriod(
    // StatusFrameEnhanced.Status_1_General,
    // STATUS_FRAME_GENERAL_PERIOD_MS,
    // CAN_TIMEOUT_MS);
    // m_turningEncoder.setStatusFramePeriod(
    // CANCoderStatusFrame.SensorData,
    // STATUS_FRAME_GENERAL_PERIOD_MS,
    // CAN_TIMEOUT_MS);
    // m_driveMotor.setStatusFramePeriod(
    // StatusFrameEnhanced.Status_4_AinTempVbat,
    // STATUS_FRAME_GENERAL_PERIOD_MS,
    // CAN_TIMEOUT_MS);
    // TODO: CAN Utilization issues

    // m_driveMotor.configSelectedFeedbackCoefficient(1);
    configMotorPID(m_turningMotor, 0, i_kPModuleTurningController, 0.0, 0.1);
    configMotorPID(m_driveMotor, 0, i_kPModuleDriveController, 0.0, 0.0);
    // System.out.println(m_turningMotor.getSelectedSensorPosition());
  }

  private double getTurningEncoderRadians() {
    return m_turningMotor.getSelectedSensorPosition(0) * 2.0 * Math.PI / i_integratedEncoderTicksPerModRev;
  }

  public void configTurningMotor() {
    // m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
    // 0, CAN_TIMEOUT_MS);
    // m_turningMotor.configSelectedFeedbackCoefficient(1.0);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setInverted(true);
    //m_turningMotor.selectProfileSlot(0, 0);
    m_turningMotor.setSelectedSensorPosition(m_turningEncoder.getAbsolutePosition() / 360.0 * i_integratedEncoderTicksPerModRev);
    m_turningMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 1.0));
  }
  public void checkPowerFailure(){
    if(m_driveMotor.hasResetOccurred()){
      System.out.println("Drive Motor" + Integer.toString(m_driveMotor.getDeviceID()));
    }
    if(m_turningMotor.hasResetOccurred()){
      System.out.println("Turning Motor" + Integer.toString(m_turningMotor.getDeviceID()));
    }
    if(m_turningEncoder.hasResetOccurred()){
      System.out.println("CANcoder" + Integer.toString(m_turningEncoder.getDeviceID()));
    }
  }
  public void configDriveMotor(){
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_driveMotor.configVoltageCompSaturation(i_kNominalVoltage);
    m_driveMotor.enableVoltageCompensation(true);
  }

  public void configStatusFramePeriods(){
    // m_turningEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
    // m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    // m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    // m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    // m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    // m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    // m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    // m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    // m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity() * i_kDriveEncoderDistancePerPulse * 10,
        new Rotation2d(getTurningEncoderRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    RezeroTurningMotorEncoder();
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = optimize(
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderRadians())),
        new Rotation2d(getTurningEncoderRadians()));

    // Calculate Arbitrary Feed Forward for Drive Motor
    final double feedForward = DriveConstants.driveTrainFeedforward.calculate(state.speedMetersPerSecond)
        / i_kNominalVoltage;

    m_driveMotor.set(ControlMode.Velocity,
        state.speedMetersPerSecond / 10.0 / i_kDriveEncoderDistancePerPulse,
        DemandType.ArbitraryFeedForward,
        feedForward);
    setHeading(state.angle.getDegrees());
  }

  public void configMotorPID(WPI_TalonFX talon, int slotIdx, double p, double i, double d) {
    talon.config_kP(slotIdx, p, CAN_TIMEOUT_MS);
    talon.config_kI(slotIdx, i, CAN_TIMEOUT_MS);
    talon.config_kD(slotIdx, d, CAN_TIMEOUT_MS);
  }

  // Zeros all the SwerveModule encoders.
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    m_turningMotor.setSelectedSensorPosition(0);
  }

  public void setHeading(double _angle) {
    double currentSensorPosition = m_turningMotor.getSelectedSensorPosition() * 360.0
        / i_integratedEncoderTicksPerModRev;
    double remainder = Math.IEEEremainder(currentSensorPosition, 360.0);
    double newAngleDemand = _angle + currentSensorPosition - remainder;

    if (newAngleDemand - currentSensorPosition > 180.1) {
      newAngleDemand -= 360.0;
    } else if (newAngleDemand - currentSensorPosition < -180.1) {
      newAngleDemand += 360.0;
    }

    m_turningMotor.set(ControlMode.Position, newAngleDemand / 360.0 * i_integratedEncoderTicksPerModRev);
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    while (Math.abs(delta.getDegrees()) > 90.0) {
      desiredState = new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          Rotation2d.fromDegrees(
              delta.getDegrees() < 90.0 ? delta.getDegrees() + 180.0 : delta.getDegrees() - 180.0));
      delta = desiredState.angle.minus(currentAngle);
    }
    return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);

  }

  private void RezeroTurningMotorEncoder() {
    if (resetIterations < 11) {
      if (resetIterations == 10) {
        m_turningMotor.setSelectedSensorPosition(
            m_turningEncoder.getAbsolutePosition() / 360.0 * i_integratedEncoderTicksPerModRev);
        System.out.println("we are bad" + Integer.toString(dub));
      }
      resetIterations++;
    }
  }
}
