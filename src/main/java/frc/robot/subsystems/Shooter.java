// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RPMConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VBusConstants;
import frc.robot.utilities.ShooterTable;
import frc.robot.utilities.ShooterTableEntry;

public class Shooter extends SubsystemBase {
  private TalonFX _front;
  private TalonFX _back;
  private TalonFX _kicker;

  private CANSparkMax _angle;
  private RelativeEncoder _angleEnc;
  /** Creates a new Shooter. */
  private static Shooter _instance = new Shooter();
  private Limelight _l;
  private ShooterTable _st = ShooterTable.getPrimaryTable();
  double limelightDistance, shooterIndex = IndexConstants.kIndexDefault;
  boolean fineAdjustment = false;
  boolean accept = true;

  public double getLimelightDistance() {
    _l.putTargetValues();
    limelightDistance = _l.distance() / 12;
    return limelightDistance;
  }

  public void toggle() {
    fineAdjustment = !fineAdjustment;
  }

  public boolean getFineAdjustment() {
    SmartDashboard.putString("Adjustment Mode", (fineAdjustment ? "Fine" : "Coarse"));
    return fineAdjustment;
  }

  public static Shooter getInstance() {
    return _instance;
  }

  public void acceptLimelight() {
    if (accept) {
      shooterIndex = limelightDistance; // TODO: round
    } else {
      shooterIndex = IndexConstants.kIndexDefault;
    }
    accept = !accept;
    SmartDashboard.putString("Accept Limelight Mode", (accept ? "Accept Limelight" : "Reset to Default"));
    // TODO: better naming
  }

  public Shooter() {
    _front = new TalonFX(SubsystemConstants.SHOOTER_FRONT_MOTOR_ID);
    _back = new TalonFX(SubsystemConstants.SHOOTER_BACK_MOTOR_ID);
    _kicker = new TalonFX(SubsystemConstants.KICKER_MOTOR_ID);

    _angle = new CANSparkMax(SubsystemConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);

    _back.setInverted(InvertType.InvertMotorOutput);

    _l = Limelight.getInstance();

    _front.configFactoryDefault();
    _back.configFactoryDefault();
    _kicker.configFactoryDefault();

    _angle.restoreFactoryDefaults();
    _angle.setSmartCurrentLimit(20);
    
    _angleEnc = _angle.getEncoder();
    // TODO: setting position using amperage
    _angleEnc.setPosition(0.);
    // _angle.getOutputCurrent();

    _front.config_kF(0, PIDConstants.Front.kF);
    _front.config_kP(0, PIDConstants.Front.kP);
    _front.config_kD(0, PIDConstants.Front.kD);

    _back.config_kF(0, PIDConstants.Back.kF);
    _back.config_kP(0, PIDConstants.Back.kP);
    _back.config_kD(0, PIDConstants.Back.kD);
  }

  public void update() {
    // Data values needed for debug
    put("Shooter Index", shooterIndex);
    ShooterTableEntry e = _st.CalcShooterValues(shooterIndex);
    SmartDashboard.putString("Shot", e.Description);
    put("Shot Front RPM", e.ShooterFrontRPM);
    put("Shot Back RPM", e.ShooterBackRPM);
    put("Actuator Value", e.ActuatorVal);

    put("Front Error", util.toFalconRPM(_front.getClosedLoopError()));
    put("Back Error", util.toFalconRPM(_back.getClosedLoopError()));
    put("Front Motor RPM", util.toFalconRPM(_front.getSelectedSensorVelocity()));
    put("Back Motor RPM", util.toFalconRPM(_back.getSelectedSensorVelocity()));

    getLimelightDistance();
  }

  public void put(String key, double val) {
    SmartDashboard.putNumber(key, val);
  }

  public void runShooterMotors() {
    ShooterTableEntry entry = ShooterTable.getPrimaryTable().CalcShooterValues(shooterIndex);
    _front.set(ControlMode.Velocity, util.toFalconVelocity(entry.ShooterFrontRPM));
    _back.set(ControlMode.Velocity, util.toFalconVelocity(entry.ShooterBackRPM));

    if (_angleEnc.getPosition() - EncoderConstants.kAngleThreshold > entry.ActuatorVal) {
      _angle.set(-VBusConstants.kAngle);
    } else if (_angleEnc.getPosition() + EncoderConstants.kAngleThreshold < entry.ActuatorVal) {
      _angle.set(VBusConstants.kAngle);
    } else {
      _angle.set(0.);
    }
  }

  public void kick() {
    _kicker.set(ControlMode.PercentOutput, VBusConstants.kKicker);
  }

  public void stopKicker() {
    _kicker.set(ControlMode.PercentOutput, 0.);
  }

  public void stop() {
    _front.set(ControlMode.PercentOutput, 0);
    _back.set(ControlMode.PercentOutput, 0);
    _kicker.set(ControlMode.PercentOutput, 0);
  }

  public double index() {
    return shooterIndex;
  }

  public void incrementIndex() {
    if (fineAdjustment) {
      shooterIndex += IndexConstants.kFineAdjustment;
    } else {
      shooterIndex += IndexConstants.kCoarseAdjustment;
    }
  }

  public void decrementIndex() {
    if (fineAdjustment) {
      shooterIndex -= IndexConstants.kFineAdjustment;
    } else {
      shooterIndex -= IndexConstants.kCoarseAdjustment;
    }
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
