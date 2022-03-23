// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VBusConstants;

public class Climber extends SubsystemBase {
  private static Climber _instance = new Climber();

  private Solenoid _tippy;
  private Solenoid _grippy;

  private CANSparkMax _left;
  private CANSparkMax _right;

  private RelativeEncoder _leftEncoder;
  private RelativeEncoder _rightEncoder;

  private double rightEncoderOffset = 0, leftEncoderOffset = 0;
  private boolean leftEncoderFinished = false, rightEncoderFinished = false;

  /** Creates a new Climber. */
  public Climber() {
    _tippy = new Solenoid(PneumaticsModuleType.CTREPCM, SubsystemConstants.TIPPY_SOLENOID_ID);
    _grippy = new Solenoid(PneumaticsModuleType.CTREPCM, SubsystemConstants.GRIPPY_SOLENOID_ID);

    _left = new CANSparkMax(SubsystemConstants.CLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
    _right = new CANSparkMax(SubsystemConstants.CLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);

    _left.restoreFactoryDefaults();
    _right.restoreFactoryDefaults();

    _left.setSmartCurrentLimit(CurrentLimitConstants.kClimber);
    _right.setSmartCurrentLimit(CurrentLimitConstants.kClimber);

    _left.setIdleMode(IdleMode.kBrake);
    _right.setIdleMode(IdleMode.kBrake);

    _left.setOpenLoopRampRate(0.1);
    _right.setOpenLoopRampRate(0.1);

    _leftEncoder = _left.getEncoder();
    _rightEncoder = _right.getEncoder();

    _leftEncoder.setPosition(0.);
    resetLeftEncoder();

    _rightEncoder.setPosition(0.);
    resetRightEncoder();
  }

  public void toggleTippySolenoid() {
    _tippy.set(!_tippy.get());
  }

  public void toggleGrippySolenoid() {
    _grippy.set(!_grippy.get());
  }

  public double getLeftEncoderPosition() {
    return (_leftEncoder.getPosition() - leftEncoderOffset);
  }

  public double getRightEncoderPosition() {
    return (_rightEncoder.getPosition() - rightEncoderOffset);
  }

  public void resetLeftEncoder() {
    leftEncoderOffset = _leftEncoder.getPosition();
  }

  public void resetRightEncoder() {
    rightEncoderOffset = _rightEncoder.getPosition();
  }

  public void leftMotorForward(double speed) {
    _left.set(speed);
    System.out.println((_leftEncoder.getPosition() - leftEncoderOffset));
  }

  public void leftMotorBackward(double speed) {
    _left.set(speed);
    System.out.println((_leftEncoder.getPosition() - leftEncoderOffset));
  }

  public void rightMotorForward(double speed) {
    _right.set(speed);
    System.out.println(_rightEncoder.getPosition());
  }

  public void rightMotorBackward(double speed) {
    _right.set(speed);
    System.out.println((_rightEncoder.getPosition() - rightEncoderOffset));
  }

  public void slowDrop() {
    _right.set(-VBusConstants.kClimberSlow);
    _left.set(-VBusConstants.kClimberSlow);
    System.out.println((_leftEncoder.getPosition() - leftEncoderOffset));
  }

  public void leftMotorOff() {
    _left.set(0.);
  }

  public void rightMotorOff() {
    _right.set(0.);
  }

  public boolean getLeftEncoderFinished() {
    return leftEncoderFinished;
  }

  public boolean getRightEncoderFinished() {
    return rightEncoderFinished;
  }

  public void setLeftEncoderFinished(boolean value) {
    leftEncoderFinished = value;
  }

  public void setRightEncoderFinished(boolean value) {
    rightEncoderFinished = value;
  }

  public void resetEncoders() {
    resetLeftEncoder();
    resetRightEncoder();
  }

  public static Climber getInstance() {
    return _instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climber", _leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Climber", _rightEncoder.getPosition());
    SmartDashboard.putNumber("Right Climber Current", _right.getOutputCurrent());
    SmartDashboard.putNumber("Left Climber Current", _left.getOutputCurrent());
  }
}
