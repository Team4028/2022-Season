// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VBusConstants;

public class Infeed extends SubsystemBase {
    /** Creates a new SingulatorAndInfeed. */
    private TalonSRX m_infeedMotor;
    private CANSparkMax m_singulatorMotor;

    private Solenoid m_infeedSolenoid;
    
    private static Infeed _instance = new Infeed();
    private boolean m_isInfeedDown = false;

    public static Infeed getInstance() {
        return _instance;
    }

    public Infeed() {
        m_infeedMotor = new TalonSRX(SubsystemConstants.INFEED_MOTOR_ID);
        m_singulatorMotor = new CANSparkMax(SubsystemConstants.SINGULATOR_MOTOR_ID, MotorType.kBrushless);
        m_infeedSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, SubsystemConstants.INFEED_SOLENOID_ID);
        m_infeedMotor.configFactoryDefault();
        m_infeedMotor.setInverted(true);
        m_singulatorMotor.setInverted(false);

        m_infeedMotor.setInverted(true);

        configStatusFramePeriods();
    }

    public void configStatusFramePeriods() {
        m_infeedMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 19);
        m_infeedMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 19);
        m_infeedMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 253);
        m_infeedMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 59);

        m_singulatorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 11);
        m_singulatorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 23);
        m_singulatorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 23);
        m_singulatorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 53);
    }

    public void runInfeedSingulatorMotors(double mult) {
        if (m_isInfeedDown) {
            m_infeedMotor.set(ControlMode.PercentOutput, mult * VBusConstants.kInfeed);
            SmartDashboard.putBoolean("Infeed/Running", true);
            // SmartDashboard.putNumber("Infeed/Vbus", mult * VBusConstants.kInfeed);
        } else {
            m_infeedMotor.set(ControlMode.PercentOutput, 0.);
            SmartDashboard.putBoolean("Infeed/Running", false);
        }
        m_singulatorMotor.set(mult * VBusConstants.kSingulator);
        SmartDashboard.putBoolean("Singulator/Running", true);
        // SmartDashboard.putNumber("Singulator/Vbus", mult * VBusConstants.kInfeed);
    }

    public void runSingulator() {
        m_singulatorMotor.set(VBusConstants.kSingulator);
        SmartDashboard.putBoolean("Singulator/Running", true);
    }

    public void forceRunInfeed() {
        m_infeedMotor.set(ControlMode.PercentOutput, 0.85);
        m_singulatorMotor.set(VBusConstants.kSingulator);
    }

    public void stopInfeedSingulatorMotors() {
        m_infeedMotor.set(ControlMode.PercentOutput, 0);
        m_singulatorMotor.set(0);
        SmartDashboard.putBoolean("Infeed/Running", false);
        SmartDashboard.putBoolean("Singulator/Running", false);
    }

    public void runInfeedMotor(double mult) {
        if (m_isInfeedDown) {
            m_infeedMotor.set(ControlMode.PercentOutput, mult * VBusConstants.kInfeed);
            SmartDashboard.putBoolean("Infeed/Running", true);
            // SmartDashboard.putNumber("Infeed/Vbus", mult * VBusConstants.kInfeed);
        } else {
            m_infeedMotor.set(ControlMode.PercentOutput, 0.);
            SmartDashboard.putBoolean("Infeed/Running", false);
        }
    }

    public void toggleInfeedUp() {
        m_isInfeedDown = !m_infeedSolenoid.get();
        m_infeedSolenoid.toggle();
    }

    public void setInfeedDown() {
        m_infeedSolenoid.set(true);
        m_isInfeedDown = true;
    }

    public void setInfeedUp() {
        m_infeedSolenoid.set(false);
        m_isInfeedDown = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
