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
    private CANSparkMax _conveyorMotor; // needs to be reversed
    private boolean isTargetReached = false;
    private RelativeEncoder _enc;
    private double encoderOffset = 0;
    private static Conveyor _instance;
    private boolean isRunning = false;

    public static Conveyor getInstance() {
        if (_instance == null) {
            _instance = new Conveyor();
        }
        return _instance;
    }

    public Conveyor() {
        _conveyorMotor = new CANSparkMax(SubsystemConstants.CONVEYOR_MOTOR_ID, MotorType.kBrushless);
        _conveyorMotor.setSmartCurrentLimit(CurrentLimitConstants.kConveyor);
        _enc = _conveyorMotor.getEncoder();
        _enc.setPosition(0);
        _conveyorMotor.setInverted(false);

        configStatusFramePeriods();
    }

    public void configStatusFramePeriods() {
        _conveyorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 11);
        _conveyorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 23);
        _conveyorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 23);
        _conveyorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 53);
    }

    public void runConveyorMotor(double vbus) {
        SmartDashboard.putBoolean("Conveyor/Running", true);
        // SmartDashboard.putNumber("Conveyor/Vbus", vbus);
        _conveyorMotor.set(vbus);
    }

    public void stopConveyorMotor() {
        _conveyorMotor.set(0);
        SmartDashboard.putBoolean("Conveyor/Running", false);
    }

    public void runConveyorMotorWithEncoder(double target, double vbus) {
        // isTargetReached = false;
        // System.out.println("runConveyorMotorWithEncoder On" + _enc.getPosition());
        if (getEncoderPosition() >= target /* && !isTargetReached */) {
            // System.out.println("Encoder Value " +_enc.getPosition());
            stopConveyorMotor();
            resetEncoder();
            isTargetReached = true;
        } else {
            runConveyorMotor(vbus);
        }
    }

    public double getEncoderPosition() {
        return (_enc.getPosition() - encoderOffset);
    }

    public void resetEncoder() {
        encoderOffset = _enc.getPosition();
    }

    public void setIsTargetReached() {
        isTargetReached = false;
    }

    public boolean getIsTargetReached() {
        return isTargetReached;
    }

    public boolean getIsRunning() {
        return isRunning;
    }

    public void setIsRunning(boolean set) {
        isRunning = set;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
