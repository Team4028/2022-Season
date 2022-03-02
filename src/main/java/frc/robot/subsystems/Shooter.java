// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.RPMConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.utilities.ShooterTable;
import frc.robot.utilities.ShooterTableEntry;

public class Shooter extends SubsystemBase {
  private TalonFX _front;
  private TalonFX _back;
  /** Creates a new Shooter. */
  private static Shooter _instance = new Shooter();
  private Limelight _l;
  private ShooterTable _st = ShooterTable.getPrimaryTable();
  double limelightDistance, shooterIndex = 6;
  boolean fineAdjustment = false;
  boolean accept = true;

  public double front_kF, front_kP,
      front_kD, front_kMax;

  public double back_kF, back_kP,
      back_kD, back_kMax;

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

    _back.setInverted(InvertType.InvertMotorOutput);

    _l = Limelight.getInstance();

    _front.configFactoryDefault();
    _back.configFactoryDefault();

    // "ok so... let's like, not ever change these numbers, ever"
    // - Gabe
    // EXCEPT when new shoter hardware
    // TODO: constants
    front_kF = 0.05;
    //front_kF = 0.03;
    back_kF = 0.055;

    front_kP = 0.4;
    back_kP = 0.1;

    // TODO: we probably don't need this at all
    // but me and Gabe feel better knowing we have this
    front_kD = 0.0;
    back_kD = 0.;

    front_kMax = 20400;
    back_kMax = 17000;

    _front.config_kF(0, front_kF);
    _back.config_kF(0, back_kF);

    _front.config_kD(0, front_kD);
    _back.config_kD(0, back_kD);

    SmartDashboard.putNumber("Front P Gain", front_kP);
    SmartDashboard.putNumber("Front D Gain", front_kD);
    SmartDashboard.putNumber("Front F Gain", front_kF);

    SmartDashboard.putNumber("Back P Gain", back_kP);
    SmartDashboard.putNumber("Back D Gain", back_kD);
    SmartDashboard.putNumber("Back F Gain", back_kF);
  }

  public static class Shot {
    public double speed;
    public double actuatorPosition;

    public Shot(double spd, double actPos) {
      speed = spd;
      actuatorPosition = actPos;
    }

    public static Shot zeroShot = new Shot(0, .3);

    // public static Shot getStopShot(){
    // return new Shot(0, getInstance()._linearActuator.get());
    // }
  }

  public void update() {
    double front_p = SmartDashboard.getNumber("Front P Gain", 0);
    double front_d = SmartDashboard.getNumber("Front D Gain", 0);
    double front_f = SmartDashboard.getNumber("Front F Gain", 0);

    if ((front_p != front_kP)) {
      _front.config_kP(0, front_p);
      front_kP = front_p;
    }
    if ((front_d != front_kD)) {
      _front.config_kD(0, front_d);
      front_kD = front_d;
    }
    if ((front_f != front_kF)) {
      _front.config_kF(0, front_f);
      front_kF = front_f;
    }

    double back_p = SmartDashboard.getNumber("Back P Gain", 0);
    double back_d = SmartDashboard.getNumber("Back D Gain", 0);
    double back_f = SmartDashboard.getNumber("Back F Gain", 0);

    if ((back_p != back_kP)) {
      _back.config_kP(0, back_p);
      back_kP = back_p;
    }
    if ((back_d != back_kD)) {
      _back.config_kD(0, back_d);
      back_kD = back_d;
    }
    if ((back_f != back_kF)) {
      _back.config_kF(0, back_f);
      back_kF = back_f;
    }

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

  // runShooterMotors... but AWESOME!
  public void runShooterMotors2() {
    ShooterTableEntry entry = ShooterTable.getPrimaryTable().CalcShooterValues(shooterIndex);
    _front.set(ControlMode.Velocity, util.toFalconVelocity(entry.ShooterFrontRPM));
    _back.set(ControlMode.Velocity, util.toFalconVelocity(entry.ShooterBackRPM));
  }

  public void runShooterMotors() {
    _front.set(ControlMode.Velocity, util.toFalconVelocity(RPMConstants.kShooterFront));
    _back.set(ControlMode.Velocity, util.toFalconVelocity(RPMConstants.kShooterBack));

    SmartDashboard.putNumber("Front Error", _front.getClosedLoopError());
    SmartDashboard.putNumber("Back Error", _back.getClosedLoopError());
    SmartDashboard.putNumber("Front Motor RPM", _front.getSelectedSensorVelocity() * 600 / 4096);
    SmartDashboard.putNumber("Back Motor RPM", _back.getSelectedSensorVelocity() * 600 / 4096);
  }

  public void stopShooterMotors() {
    _front.set(ControlMode.PercentOutput, 0);
    _back.set(ControlMode.PercentOutput, 0);
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
