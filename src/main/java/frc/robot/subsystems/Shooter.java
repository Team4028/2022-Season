// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.utilities.ShooterIndex;
import frc.robot.utilities.ShooterTable;
import frc.robot.utilities.ShooterTableEntry;

public class Shooter extends SubsystemBase {
    private TalonFX m_frontMotor;
    private TalonFX m_backMotor;
    private TalonFX m_kickerMotor;

    private CANSparkMax m_angleMotor;
    private RelativeEncoder m_angleEncoder;
    private SparkMaxPIDController m_anglePID;

    private boolean isShotValidation;

    private ShooterTable m_table = ShooterTable.getPrimaryTable();
    double limelightDistance, manualIndex, shooterIndex = ShooterConstants.kIndexDefault;
    int manualCounter = 0;
    boolean longshot;

    private double lastIndex = 0.;

    private ShooterIndex indexData;

    private static Shooter _instance;
    private int updateCycles = 0;

    private boolean m_isAtSetpoint = false;
    private boolean m_isVbus = true;

    public Shooter() {
        m_frontMotor = new TalonFX(SubsystemConstants.SHOOTER_FRONT_MOTOR_ID);
        m_backMotor = new TalonFX(SubsystemConstants.SHOOTER_BACK_MOTOR_ID);
        m_kickerMotor = new TalonFX(SubsystemConstants.KICKER_MOTOR_ID);

        m_angleMotor = new CANSparkMax(SubsystemConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);

        m_backMotor.setInverted(false);
        m_frontMotor.setInverted(false);
        m_kickerMotor.setInverted(InvertType.InvertMotorOutput);

        m_frontMotor.setNeutralMode(NeutralMode.Coast);
        m_backMotor.setNeutralMode(NeutralMode.Coast);
        m_kickerMotor.setNeutralMode(NeutralMode.Coast);

        // _front.configFactoryDefault();
        // _back.configFactoryDefault();
        // _kicker.configFactoryDefault();

        m_angleMotor.restoreFactoryDefaults();
        m_angleMotor.setSmartCurrentLimit(CurrentLimitConstants.kAngle);
        m_angleMotor.setInverted(true);
        m_angleMotor.setIdleMode(IdleMode.kCoast);

        m_angleEncoder = m_angleMotor.getEncoder();
        m_angleEncoder.setPosition(0.);

        m_anglePID = m_angleMotor.getPIDController();
        m_anglePID.setP(PIDConstants.Angle.kP);

        // put("FrontVbus", VBusConstants.kShooterFrontDefault);
        // put("BackVbus", VBusConstants.kShooterBackDefault);
        // put("Hood Angle (rot)", VBusConstants.kShooterHoodAngleRotDefault);

        m_frontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100);
        m_backMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100);

        isShotValidation = false;

        indexData = new ShooterIndex();
        indexData.setFontSize(100);

        configStatusFramePeriods();
    }

    public void configStatusFramePeriods() {
        m_angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 11);
        m_angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 23);
        m_angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 23);
        m_angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 53);

        m_frontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 19);
        m_frontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 19);
        m_frontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 253);
        m_frontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 59);

        m_backMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 19);
        m_backMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 19);
        m_backMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 253);
        m_backMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 59);

        m_kickerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 19);
        m_kickerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 19);
        m_kickerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 253);
        m_kickerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 59);

    }

    /**
     * @param vbus true if vbus, false if PID
     */
    public void setIsVbus(boolean vbus) {
        m_isVbus = vbus;
        if (vbus) {
            m_frontMotor.configVoltageCompSaturation(ShooterConstants.kVoltageCompensation);
            m_backMotor.configVoltageCompSaturation(ShooterConstants.kVoltageCompensation);
            m_kickerMotor.configVoltageCompSaturation(ShooterConstants.kVoltageCompensation);

            m_table = ShooterTable.getPrimaryTable();
        } else {
            m_frontMotor.config_kF(0, PIDConstants.Front.kF);
            m_frontMotor.config_kP(0, PIDConstants.Front.kP);
            m_frontMotor.config_kD(0, PIDConstants.Front.kD);

            m_backMotor.config_kF(0, PIDConstants.Back.kF);
            m_backMotor.config_kP(0, PIDConstants.Back.kP);
            m_backMotor.config_kD(0, PIDConstants.Back.kD);

            m_table = ShooterTable.getSecondaryTable();
        }
    }

    public boolean getIsVbus() {
        return m_isVbus;
    }

    public void toggleIsVbus() {
        setIsVbus(!m_isVbus);
    }

    public void put(String key, double val) {
        SmartDashboard.putNumber(key, val);
    }

    public double getNumber(String key, double defaultValue) {
        return SmartDashboard.getNumber(key, defaultValue);
    }
    // _front.set(ControlMode.PercentOutput, SmartDashboard.getNumber("FrontVbus",
    // VBusConstants.kShooterFrontDefault));
    // _back.set(ControlMode.PercentOutput, SmartDashboard.getNumber("BackVbus",
    // VBusConstants.kShooterBackDefault));
    // _kicker.set(ControlMode.PercentOutput, SmartDashboard.getNumber("KickerVbus",
    // VBusConstants.kKicker));

    // _anglePid.setReference(SmartDashboard.getNumber("Hood Angle (rot)",
    // VBusConstants.kShooterHoodAngleRotDefault), ControlType.kPosition);

    public void runShooterMotors() {
        ShooterTableEntry entry = m_table.CalcShooterValues(shooterIndex);

        if (m_isVbus) {
            m_frontMotor.set(ControlMode.PercentOutput, entry.ShooterFrontRPM / 100.);
            m_backMotor.set(ControlMode.PercentOutput, entry.ShooterBackRPM / 100.);
        } else {
            m_frontMotor.set(ControlMode.Velocity, util.toFalconVelocity(entry.ShooterFrontRPM));
            m_backMotor.set(ControlMode.Velocity, util.toFalconVelocity(entry.ShooterBackRPM));

            m_isAtSetpoint = 
                Math.abs(util.toFalconRPM(m_frontMotor.getClosedLoopError())) < 100. &&
                Math.abs(util.toFalconRPM(m_backMotor.getClosedLoopError())) < 100.;
        }

        m_kickerMotor.set(ControlMode.PercentOutput, entry.KickerRPM / 100.);
        m_anglePID.setReference(entry.ActuatorVal, ControlType.kPosition);

        put("Front Motor RPM", util.toFalconRPM(m_frontMotor.getSelectedSensorVelocity()));
        put("Back Motor RPM", util.toFalconRPM(m_backMotor.getSelectedSensorVelocity()));
        put("Kicker Motor RPM", util.toFalconRPM(m_kickerMotor.getSelectedSensorVelocity()));
        put("Angle", m_angleEncoder.getPosition());

        put("Front Motor Error", (m_frontMotor.getClosedLoopError()));
        put("Back Motor Error", (m_backMotor.getClosedLoopError()));

        SmartDashboard.putBoolean("Shooter/Running", true);
    }

    public boolean getIsAtSetpoint() {
        return m_isAtSetpoint;
    }

    public void stop() {
        m_frontMotor.set(ControlMode.PercentOutput, 0);
        m_backMotor.set(ControlMode.PercentOutput, 0);
        m_kickerMotor.set(ControlMode.PercentOutput, 0);
        m_angleMotor.set(0.0);

        SmartDashboard.putBoolean("Shooter/Running", false);
    }

    public double index() {
        return shooterIndex;
    }

    public double manualIndex() {
        return manualIndex;
    }

    public void setLongshot(boolean longshot) {
        if (this.longshot != longshot) {
            resetCounter();
            incrementCounter();
        }
        this.longshot = longshot;
    }

    public void incrementIndex(boolean fine) {
        if (fine) {
            manualIndex += ShooterConstants.kFineAdjustment;
        } else {
            manualIndex += ShooterConstants.kCoarseAdjustment;
        }
        // update();
    }

    public void decrementIndex(boolean fine) {
        if (fine) {
            manualIndex -= ShooterConstants.kFineAdjustment;
        } else {
            manualIndex -= ShooterConstants.kCoarseAdjustment;
        }
        // update();
    }

    public void setShooterIndex(double index, boolean setManual) {
        if (setManual) {
            manualIndex = index;
        }
        shooterIndex = index;

        if (lastIndex != index) {
            indexData.setIndex(Math.round(index * 10.) / 10.);
            SmartDashboard.putData("Shooter Index", indexData);

            ShooterTableEntry e = m_table.CalcShooterValues(shooterIndex);
            SmartDashboard.putString("Shot", e.Description);
            put("Shot Front RPM", e.ShooterFrontRPM);
            put("Shot Back RPM", e.ShooterBackRPM);
            put("Shot Kicker RPM", e.KickerRPM);
            put("Actuator Value", e.ActuatorVal);
            lastIndex = index;
        }
    }

    public void setManualIndex() {
        if (!longshot) {
            if (manualCounter() % 3 == 0) {
                manualIndex = 7.0;
            } else if (manualCounter() % 3 == 2) {
                manualIndex = 10.0;
            } else {
                manualIndex = 12.5;
            }
        } else {
            if (manualCounter() % 3 == 0) {
                manualIndex = 21.0;
            } else if (manualCounter() % 3 == 2) {
                manualIndex = 17.0;
            } else {
                manualIndex = 15.0;
            }
        }
    }

    public void resetIndex() {
        manualIndex = ShooterConstants.kIndexDefault;
        shooterIndex = ShooterConstants.kIndexDefault;
    }

    public void resetManualIndex() {
        manualIndex = ShooterConstants.kIndexDefault;
        resetCounter();
    }

    public int manualCounter() {
        return manualCounter;
    }

    public void incrementCounter() {
        manualCounter++;
    }

    public void resetCounter() {
        manualCounter = 0;
    }

    public void setIsShotValidation(boolean isShotValidation) {
        this.isShotValidation = isShotValidation;
        SmartDashboard.putBoolean("Shot Validation", this.isShotValidation);
    }

    public void toggleIsShotValidation() {
        this.isShotValidation = !this.isShotValidation;
        SmartDashboard.putBoolean("Shot Validation", this.isShotValidation);
    }

    public boolean getIsShotValidation() {
        return isShotValidation;
    }

    public void runShooterOutfeed() {
        m_frontMotor.set(ControlMode.PercentOutput, 0.35);
        m_backMotor.set(ControlMode.PercentOutput, 0);
        m_kickerMotor.set(ControlMode.PercentOutput, 0.55);
        m_anglePID.setReference(25.0, ControlType.kPosition);
    }

    public static Shooter getInstance() {
        if (_instance == null) {
            _instance = new Shooter();
        }
        return _instance;
    }

    @Override
    public void periodic() {
        if (updateCycles % 5 == 0) {
            // update();
            // SmartDashboard.putNumber("Manual Index", manualIndex());
            SmartDashboard.putBoolean("Shot Validation", getIsShotValidation());
        }
        updateCycles++;
        // This method will be called once per scheduler run
        // System.out.println(_angle.getOutputCurrent());

    }
}
