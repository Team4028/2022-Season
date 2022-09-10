// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.RampRateConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VBusConstants;

public class Climber extends SubsystemBase {
    private static Climber _instance = new Climber();

    private Solenoid m_latchSolenoid;
    private Solenoid m_grippySolenoid;

    private CANSparkMax m_leftMotor;
    private CANSparkMax m_rightMotor;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    private int m_updateCycles = 0;

    /** Creates a new cry about it. */
    public Climber() {
        m_latchSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, SubsystemConstants.TIPPY_SOLENOID_ID);
        m_grippySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, SubsystemConstants.GRIPPY_SOLENOID_ID);

        m_leftMotor = new CANSparkMax(SubsystemConstants.CLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(SubsystemConstants.CLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_leftMotor.setSmartCurrentLimit(CurrentLimitConstants.kClimber);
        m_rightMotor.setSmartCurrentLimit(CurrentLimitConstants.kClimber);

        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setIdleMode(IdleMode.kBrake);

        m_leftMotor.setOpenLoopRampRate(RampRateConstants.kClimber);
        m_rightMotor.setOpenLoopRampRate(RampRateConstants.kClimber);

        m_leftEncoder = m_leftMotor.getEncoder();
        m_rightEncoder = m_rightMotor.getEncoder();

        resetLeftEncoder();
        resetRightEncoder();

        configStatusFramePeriods();
    }

    public void configStatusFramePeriods() {
        m_leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 11);
        m_leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 23);
        m_leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 23);
        m_leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 53);

        m_rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 11);
        m_rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 23);
        m_rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 23);
        m_rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 53);
    }

    public void toggleLatchSolenoid() {
        m_latchSolenoid.toggle();
    }

    public void toggleGrippySolenoid() {
        m_grippySolenoid.toggle();
    }

    public double getLeftEncoderPosition() {
        return m_leftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return m_rightEncoder.getPosition();
    }

    public void resetLeftEncoder() {
        m_leftEncoder.setPosition(0.);
    }

    public void resetRightEncoder() {
        m_rightEncoder.setPosition(0.);
    }

    public void leftMotorForward(double speed) {
        m_leftMotor.set(speed);
    }

    public void leftMotorBackward(double speed) {
        m_leftMotor.set(speed);
    }

    public void rightMotorForward(double speed) {
        m_rightMotor.set(speed);
    }

    public void rightMotorBackward(double speed) {
        m_rightMotor.set(speed);
    }

    public void slowDrop() {
        m_rightMotor.set(-VBusConstants.kClimberSlow);
        m_leftMotor.set(-VBusConstants.kClimberSlow);
    }

    public void slowUp() {
        m_rightMotor.set(VBusConstants.kClimberSlow);
        m_leftMotor.set(VBusConstants.kClimberSlow);
    }

    public void leftMotorOff() {
        m_leftMotor.set(0.);
    }

    public void rightMotorOff() {
        m_rightMotor.set(0.);
    }

    public void resetEncoders() {
        resetLeftEncoder();
        resetRightEncoder();
    }

    public void setRightEncoder(double val) {
        m_rightEncoder.setPosition(val);
    }

    public void setLeftEncoder(double val) {
        m_leftEncoder.setPosition(val);
    }

    public void stop() {
        rightMotorOff();
        leftMotorOff();
    }

    public void setCoast() {
        m_leftMotor.setIdleMode(IdleMode.kCoast);
        m_rightMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setBrake() {
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setIdleMode(IdleMode.kBrake);
    }

    public double rightCurrent() {
        return m_rightMotor.getOutputCurrent();
    }

    public double leftCurrent() {
        return m_leftMotor.getOutputCurrent();
    }

    public static Climber getInstance() {
        return _instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_updateCycles % 4 == 0) {
            SmartDashboard.putNumber("Left Climber", m_leftEncoder.getPosition());
            SmartDashboard.putNumber("Right Climber", m_rightEncoder.getPosition());
        }
        m_updateCycles++;
    }
}
