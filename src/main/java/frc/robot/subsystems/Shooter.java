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
    private TalonFX _front;
    private TalonFX _back;
    private TalonFX _kicker;

    private CANSparkMax _angle;
    private RelativeEncoder _angleEnc;
    private SparkMaxPIDController _anglePid;

    private boolean isShotValidation;

    private Limelight _l;
    private ShooterTable _st = ShooterTable.getPrimaryTable();
    double limelightDistance, manualIndex, shooterIndex = ShooterConstants.kIndexDefault;
    int manualCounter = 0;
    boolean longshot;

    private double lastIndex = 0.;

    private ShooterIndex indexData;

    private static Shooter _instance;
    private int updateCycles = 0;

    public Shooter() {
        _front = new TalonFX(SubsystemConstants.SHOOTER_FRONT_MOTOR_ID);
        _back = new TalonFX(SubsystemConstants.SHOOTER_BACK_MOTOR_ID);
        _kicker = new TalonFX(SubsystemConstants.KICKER_MOTOR_ID);

        _angle = new CANSparkMax(SubsystemConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);

        _back.setInverted(false);
        _front.setInverted(false);
        _kicker.setInverted(InvertType.InvertMotorOutput);

        _front.setNeutralMode(NeutralMode.Coast);
        _back.setNeutralMode(NeutralMode.Coast);
        _kicker.setNeutralMode(NeutralMode.Coast);

        _front.configVoltageCompSaturation(ShooterConstants.kVoltageCompensation);
        _front.enableVoltageCompensation(ShooterConstants.kUseVoltageComp);

        _back.configVoltageCompSaturation(ShooterConstants.kVoltageCompensation);
        _back.enableVoltageCompensation(ShooterConstants.kUseVoltageComp);

        _kicker.configVoltageCompSaturation(ShooterConstants.kVoltageCompensation);
        _kicker.enableVoltageCompensation(ShooterConstants.kUseVoltageComp);

        _l = Limelight.getInstance();

        // _front.configFactoryDefault();
        // _back.configFactoryDefault();
        // _kicker.configFactoryDefault();

        _angle.restoreFactoryDefaults();
        _angle.setSmartCurrentLimit(CurrentLimitConstants.kAngle);
        _angle.setInverted(true);
        _angle.setIdleMode(IdleMode.kCoast);

        _angleEnc = _angle.getEncoder();
        _angleEnc.setPosition(0.);

        _anglePid = _angle.getPIDController();
        _anglePid.setP(PIDConstants.Angle.kP);

        // put("FrontVbus", VBusConstants.kShooterFrontDefault);
        // put("BackVbus", VBusConstants.kShooterBackDefault);
        // put("Hood Angle (rot)", VBusConstants.kShooterHoodAngleRotDefault);

        if (!ShooterConstants.kIsVBus) {
            _front.config_kF(0, PIDConstants.Front.kF);
            _front.config_kP(0, PIDConstants.Front.kP);
            _front.config_kD(0, PIDConstants.Front.kD);

            _back.config_kF(0, PIDConstants.Back.kF);
            _back.config_kP(0, PIDConstants.Back.kP);
            _back.config_kD(0, PIDConstants.Back.kD);
        }

        _front.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100);
        _back.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100);

        isShotValidation = false;

        indexData = new ShooterIndex();
        indexData.setFontSize(100);

        configStatusFramePeriods();
    }

    public void configStatusFramePeriods() {
        _angle.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 11);
        _angle.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 23);
        _angle.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 23);
        _angle.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 53);

        _front.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 19);
        _front.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 19);
        _front.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 253);
        _front.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 59);

        _back.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 19);
        _back.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 19);
        _back.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 253);
        _back.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 59);

        _kicker.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 19);
        _kicker.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 19);
        _kicker.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 253);
        _kicker.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 59);

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
        ShooterTableEntry entry = ShooterTable.getPrimaryTable().CalcShooterValues(shooterIndex);

        if (ShooterConstants.kIsVBus) {
            _front.set(ControlMode.PercentOutput, entry.ShooterFrontRPM / 100.);
            _back.set(ControlMode.PercentOutput, entry.ShooterBackRPM / 100.);
            _kicker.set(ControlMode.PercentOutput, entry.KickerRPM / 100.);
        } else {
            _front.set(ControlMode.Velocity, util.toFalconVelocity(entry.ShooterFrontRPM));
            _back.set(ControlMode.Velocity, util.toFalconVelocity(entry.ShooterBackRPM));
            _kicker.set(ControlMode.PercentOutput, entry.KickerRPM / 100.);
        }

        put("Front Motor RPM", util.toFalconRPM(_front.getSelectedSensorVelocity()));
        put("Back Motor RPM", util.toFalconRPM(_back.getSelectedSensorVelocity()));
        put("Kicker Motor RPM", util.toFalconRPM(_kicker.getSelectedSensorVelocity()));
        put("Angle", _angleEnc.getPosition());

        SmartDashboard.putBoolean("Shooter/Running", true);

        _anglePid.setReference(entry.ActuatorVal, ControlType.kPosition);
    }

    public void stop() {
        _front.set(ControlMode.PercentOutput, 0);
        _back.set(ControlMode.PercentOutput, 0);
        _kicker.set(ControlMode.PercentOutput, 0);
        _angle.set(0.0);

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

            ShooterTableEntry e = _st.CalcShooterValues(shooterIndex);
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

    public void acceptLimelight() {
        setShooterIndex(_l.willTestDistance(), false);
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
        _front.set(ControlMode.PercentOutput, 0.35);
        _back.set(ControlMode.PercentOutput, 0);
        _kicker.set(ControlMode.PercentOutput, 0.55);
        _anglePid.setReference(25.0, ControlType.kPosition);
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
