// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.SubsystemConstants;

public class Conveyor extends SubsystemBase {
    /** Creates a new Conveyor. */
    private CANSparkMax m_conveyorMotor; // needs to be reversed

    private RelativeEncoder m_conveyorEncoder;

    private boolean m_isTargetReached = false;
    private double m_encoderOffset = 0;

    private static Conveyor _instance;
    private boolean m_isRunning = false;

    public static Conveyor getInstance() {
        if (_instance == null) {
            _instance = new Conveyor();
        }
        return _instance;
    }

    public Conveyor() {
        m_conveyorMotor = new CANSparkMax(SubsystemConstants.CONVEYOR_MOTOR_ID, MotorType.kBrushless);
        m_conveyorMotor.setSmartCurrentLimit(CurrentLimitConstants.kConveyor);
        m_conveyorEncoder = m_conveyorMotor.getEncoder();
        m_conveyorEncoder.setPosition(0);
        m_conveyorMotor.setInverted(false);

        configStatusFramePeriods();
    }

    public void configStatusFramePeriods() {
        m_conveyorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 11);
        m_conveyorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 23);
        m_conveyorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 23);
        m_conveyorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 53);
    }

    public void runConveyorMotor(double vbus) {
        SmartDashboard.putBoolean("Conveyor/Running", true);
        // SmartDashboard.putNumber("Conveyor/Vbus", vbus);
        m_conveyorMotor.set(vbus);
    }

    public void stopConveyorMotor() {
        m_conveyorMotor.set(0);
        SmartDashboard.putBoolean("Conveyor/Running", false);
    }

    public void runConveyorMotorWithEncoder(double target, double vbus) {
        // isTargetReached = false;
        // System.out.println("runConveyorMotorWithEncoder On" + _enc.getPosition());
        if (getEncoderPosition() >= target /* && !isTargetReached */) {
            // System.out.println("Encoder Value " +_enc.getPosition());
            stopConveyorMotor();
            resetEncoder();
            m_isTargetReached = true;
        } else {
            runConveyorMotor(vbus);
        }
    }

    public double getEncoderPosition() {
        return (m_conveyorEncoder.getPosition() - m_encoderOffset);
    }

    public void resetEncoder() {
        m_encoderOffset = m_conveyorEncoder.getPosition();
    }

    public void setIsTargetReached() {
        m_isTargetReached = false;
    }

    public boolean getIsTargetReached() {
        return m_isTargetReached;
    }

    public boolean getIsRunning() {
        return m_isRunning;
    }

    public void setIsRunning(boolean set) {
        m_isRunning = set;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
