// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  //private Motor _m = Motor.getInstance();

  private int distEstIters = 0;
  private double distEst, distEstTotal;

  private double kDistIters = 20;

  private static Limelight _instance = new Limelight();

  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = entry("tx");
  private NetworkTableEntry ta = entry("ta");
  private NetworkTableEntry tv = entry("tv");
  private NetworkTableEntry tshort = entry("tshort");
  private NetworkTableEntry tlong = entry("tlong");
  private NetworkTableEntry thor = entry("thor");
  private NetworkTableEntry tvert = entry("tvert");
  private NetworkTableEntry ty = entry("ty");
  private NetworkTableEntry ts = entry("ts");
  private NetworkTableEntry ledMode = entry("ledMode");
  private NetworkTableEntry pipeline = entry("pipeline");

  /** Creates a new Limelight. */
  public Limelight() {
    setPipeline(0);
  }

  public double getX() {
    return tx.getDouble(0);
  }
  public double getLedMode(){
    return ledMode.getDouble(0);
  }
  public void setLedMode(double mode){
    ledMode.forceSetDouble(mode);
  }
  public double getArea() {
    return ta.getDouble(0);
  }
  public double getBoxShortLength() {
    return tshort.getDouble(0);
  }
  public double getBoxLongLength(){
    return tlong.getDouble(0);
  }
  public double getHorBoxLength(){
    return thor.getDouble(0);
  }
  public double getVertBoxLength(){
    return tvert.getDouble(0);
  }
  public double getSkew() {
    return ts.getDouble(0);
  }
  public double getY() {
    return ty.getDouble(0);
  }

  public void setPipeline(double pipe) {
    pipeline.setDouble(pipe);
  }

  public boolean getHasTarget() {
    return tv.getDouble(0.0) != 0.0;
  }

  public double distance() {
    if (getHasTarget()) {
      if (distEstIters >= kDistIters) {
        distEst = distEstTotal / distEstIters;
        distEstTotal = 0;
        distEstIters = 0;
        put("Distance", distEst * 12);
      }

      // EXCEL OP
      double dist = 0.9306 * Math.pow(getArea(), -1.243);
      distEstTotal += dist;
      distEstIters++;
    }
    return distEst;
 }

  /*public void update() {
    double mountAngle = get("Mount Angle", 0.);
    double mountHeight = get("Mount Height", 0.);
    double targetHeight = get("Target Height", 0.);
    double distIters = get("Distance Iterations", 0.);

    if (mountAngle != kMountAngle) { kMountAngle = mountAngle; }
    if (mountHeight != kMountHeight) { kMountHeight = mountHeight; }
    if (targetHeight != kTargetHeight) { kTargetHeight = targetHeight; }
    if (distIters != kDistIters) { kDistIters = distIters; }
  }*/

  public void putTargetValues() {
    put("Target X Offset", entry("tx").getDouble(0.));
    put("Target Y Offset", entry("ty").getDouble(0.));
    put("Target Area", entry("ta").getDouble(0.));
  }

  public boolean set(String key, double val) {
    return entry(key).setNumber(val);
  }

  public NetworkTableEntry entry(String key) {
    return nt.getEntry(key);
  }

  public void put(String key, double val) {
    SmartDashboard.putNumber(key, val);
  }

  public double get(String key, double defaultValue) {
    return SmartDashboard.getNumber(key, defaultValue);
  }

  public static Limelight getInstance() {
    return _instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
