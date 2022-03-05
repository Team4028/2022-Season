// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.MK4IModuleConstants.*;

public class SwerveModuleCANTwoElectricBoogaloo {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;
  private final WPI_CANCoder m_turningEncoder;

  private final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private final int CAN_TIMEOUT_MS = 250;


  private int resetIterations = 0;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModuleCANTwoElectricBoogaloo(
      int driveMotorChannel,
      int turningMotorChannel,
      int CANEncoderPort,
      double turningMotorOffset) {
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    m_turningEncoder = new WPI_CANCoder(CANEncoderPort);

    // m_turningEncoder.configFactoryDefault();
    // m_turningMotor.configFactoryDefault();
    // m_driveMotor.configFactoryDefault();
    m_turningEncoder.setPositionToAbsolute();
    if(m_turningEncoder.configGetSensorInitializationStrategy() != SensorInitializationStrategy.BootToAbsolutePosition){
        m_turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, CAN_TIMEOUT_MS);
    }
    m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS);
    
    m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    m_turningMotor.configSelectedFeedbackCoefficient(1);
    m_turningEncoder.configMagnetOffset(Math.toDegrees(turningMotorOffset));
    m_turningMotor.setSensorPhase(false);
    m_turningMotor.configAllowableClosedloopError(0, 10, CAN_TIMEOUT_MS);
    m_turningMotor.setStatusFramePeriod(
              StatusFrameEnhanced.Status_1_General,
              STATUS_FRAME_GENERAL_PERIOD_MS,
              CAN_TIMEOUT_MS
      );

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_driveMotor.configSelectedFeedbackCoefficient(1);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setInverted(true);
    m_turningMotor.selectProfileSlot(0, 0);
    m_turningMotor.setSelectedSensorPosition(m_turningEncoder.getPosition() / 360 * 2048 * 150 / 7);
    m_turningMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 1.0));
    m_driveMotor.configVoltageCompSaturation(12.0);
    m_driveMotor.enableVoltageCompensation(true);
    

    configMotorPID(m_turningMotor, 0, 0.2, 0.0, 0.1);
    configMotorPID(m_driveMotor, 0, 0.13367, 0.0, 0.0);
    m_driveMotor.config_kF(0, 0);//DriveConstants.kvVoltSecondsPerMeter * 10.0 / 12.0 * MK4IModuleConstants.i_kDriveEncoderDistancePerPulse);
    System.out.println(m_turningMotor.getSelectedSensorPosition());
  }

  private double getTurningEncoderRadians(){
    return m_turningMotor.getSelectedSensorPosition(0) * 2.0 * Math.PI / i_integratedEncoderTicksPerModRev;
    }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity() * i_kDriveEncoderDistancePerPulse * 10, new Rotation2d(getTurningEncoderRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    checkMotorCoder();
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        vargooseoptimize(SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderRadians())), new Rotation2d(getTurningEncoderRadians()));

    // Calculate the drive output from the drive PID controller.
    final double feedForward =
        DriveConstants.driveTrainFeedforward.calculate(state.speedMetersPerSecond) / i_kNominalVoltage;

    //m_driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / MK4IModuleConstants.i_kDriveEncoderDistancePerPulse);
    m_driveMotor.set(ControlMode.Velocity,
    state.speedMetersPerSecond / 10.0 / i_kDriveEncoderDistancePerPulse,
    DemandType.ArbitraryFeedForward,
    feedForward);
    setHeading(state.angle.getDegrees());
  }

  public void configMotorPID(WPI_TalonFX talon, int slotIdx, double p, double i, double d){
    talon.config_kP(slotIdx, p, CAN_TIMEOUT_MS);
    talon.config_kI(slotIdx, i, CAN_TIMEOUT_MS);
    talon.config_kD(slotIdx, d, CAN_TIMEOUT_MS);
  }

//Zeros all the SwerveModule encoders.
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    m_turningMotor.setSelectedSensorPosition(0);
  }


public void setHeading(double _angle){
  //double newAngleDemand = _angle;
  double currentSensorPosition = m_turningMotor.getSelectedSensorPosition() * 360.0 / i_integratedEncoderTicksPerModRev;
  double remainder = Math.IEEEremainder(currentSensorPosition, 360.0);
  double newAngleDemand = _angle + currentSensorPosition -remainder;
 
  //System.out.println(mSteeringMotor.getSelectedSensorPosition()-remainder );
  if(newAngleDemand - currentSensorPosition > 180.1){
        newAngleDemand -= 360.0;
    } else if (newAngleDemand - currentSensorPosition < -180.1){
        newAngleDemand += 360.0;
    }
    
  m_turningMotor.set(ControlMode.Position, newAngleDemand / 360.0 * i_integratedEncoderTicksPerModRev);
}
public static SwerveModuleState vargooseoptimize(
  SwerveModuleState desiredState, Rotation2d currentAngle) {
var delta = desiredState.angle.minus(currentAngle);
while(Math.abs(delta.getDegrees()) > 90.0){
  desiredState =   new SwerveModuleState(
    -desiredState.speedMetersPerSecond,
    Rotation2d.fromDegrees(
      delta.getDegrees() < 90.0?
      delta.getDegrees() + 180.0:
      delta.getDegrees() - 180.0
    ));
  delta = desiredState.angle.minus(currentAngle);
}
  return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);

}
private void checkMotorCoder(){
  if(resetIterations <1){
    m_turningMotor.setSelectedSensorPosition(m_turningEncoder.getAbsolutePosition() / 360 * 2048 * (150/7));
  }
  resetIterations++;
}
public static SwerveModuleState optimizetwoelecboogaloo(SwerveModuleState desiredState, Rotation2d currentAngle) {
  double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
  double targetSpeed = desiredState.speedMetersPerSecond;
  double delta = targetAngle - currentAngle.getDegrees();
  if (Math.abs(delta) > 90){
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
  }        
  return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
}

/**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360 - lowerOffset);
    } else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
        newAngle += 360;
    }
    while (newAngle > upperBound) {
        newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
        newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
        newAngle += 360;
    }
    return newAngle;
}

}
