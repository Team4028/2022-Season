// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PicoColorSensor;
import frc.robot.PicoColorSensor.RawColor;
import frc.robot.utilities.BallStatus;

public class ColorSensor extends SubsystemBase {
  private PicoColorSensor m_sensor = new PicoColorSensor();
  private int redBalls = 0;
  private int blueBalls = 0;
  private boolean redBallFoundLastCycle;
  private boolean blueBallFoundLastCycle;
  private int redThreshold0 = 300;
  private int blueThreshold0 = 350;
  private int proximityThreshold0 = 120;

  private int redThreshold1 = 300;
  private int blueThreshold1 = 400;
  private int proximityThreshold1 = 180;

  private boolean redBall0 = false;
  private boolean blueBall0 = false;

  private boolean redBall1 = false;
  private boolean blueBall1 = false;

  private boolean ball0Detected;
  private boolean ball1Detected;

  private BallStatus innerBall;
  private BallStatus outerBall;

  Alliance alliance;

  private static ColorSensor _instance = new ColorSensor();
  private int updateCycles;

  /** Creates a new ColorSensor. */
  public ColorSensor() {
    updateCycles = 0;
    alliance = Alliance.Red; // Just to avoid nullpointer
    innerBall = new BallStatus();
    outerBall = new BallStatus();
  }

  public void updateSensor0() {
    RawColor rc0 = m_sensor.getRawColor0();

    int red0 = rc0.red;
    int green0 = rc0.green;
    int blue0 = rc0.blue;
    int proximity0 = m_sensor.getProximity0();

    int higher = Math.max(red0, blue0);

    boolean hasRed = false;
    boolean hasBlue = false;
    if (proximity0 > proximityThreshold0) {
      hasRed = red0 > redThreshold0 && higher == red0;
      hasBlue = blue0 > blueThreshold0 && higher == blue0;
      if (hasRed) {
        if (!redBallFoundLastCycle) {
          redBalls++;
        }
        redBallFoundLastCycle = true;
      }
      if (blue0 > blueThreshold0 && higher == blue0) {
        if (!blueBallFoundLastCycle) {
          blueBalls++;
        }
        blueBallFoundLastCycle = true;
      }
    } else {
      redBallFoundLastCycle = false;
      blueBallFoundLastCycle = false;
    }

    SmartDashboard.putBoolean("red ball 0", hasRed);
    outerBall.setRed(hasRed);
    redBall0 = hasRed;

    SmartDashboard.putBoolean("blue ball 0", hasBlue);
    outerBall.setBlue(hasBlue);
    blueBall0 = hasBlue;

    SmartDashboard.putNumber("red 0", red0);
    SmartDashboard.putNumber("blue 0", blue0);
    SmartDashboard.putNumber("proximity 0", proximity0);
    SmartDashboard.putNumber("total red balls", redBalls);
    SmartDashboard.putNumber("total blue balls", blueBalls);
    ball0Detected = proximity0 > proximityThreshold0;

    outerBall.setBall(ball0Detected);
  }

  public void updateSensor1() {
    RawColor rc1 = m_sensor.getRawColor1();

    int red1 = rc1.red;
    int green1 = rc1.green;
    int blue1 = rc1.blue;
    int proximity1 = m_sensor.getProximity1();

    int higher = Math.max(red1, blue1);

    boolean hasRed = false;
    boolean hasBlue = false;
    if (proximity1 > proximityThreshold1) {
      hasRed = red1 > redThreshold1 && higher == red1;
      hasBlue = blue1 > blueThreshold1 && higher == blue1;
    }

    SmartDashboard.putBoolean("red ball 1", hasRed);
    innerBall.setRed(hasRed);
    redBall1 = hasRed;

    SmartDashboard.putBoolean("blue ball 1", hasBlue);
    innerBall.setBlue(hasBlue);
    blueBall1 = hasBlue;

    boolean[] bruh = { blue1 > blueThreshold1 && higher == blue1, red1 > redThreshold1 && higher == red1 };
    SmartDashboard.putBooleanArray("ball0", bruh);

    SmartDashboard.putNumber("red 1", red1);
    SmartDashboard.putNumber("blue 1", blue1);
    SmartDashboard.putNumber("proximity 1", proximity1);
    ball1Detected = proximity1 > proximityThreshold1;

    innerBall.setBall(ball1Detected);
  }

  public void setAlliance() {
    alliance = DriverStation.getAlliance() != Alliance.Invalid ? DriverStation.getAlliance() : Alliance.Red;
  }

  public boolean getOuterBallCorrect() {
    return (alliance.equals(Alliance.Red) && redBall0 && ball0Detected)
        ||
        (alliance.equals(Alliance.Blue) && blueBall0 && ball0Detected);
  }

  public boolean getInnerBallCorrect() {
    return (alliance.equals(Alliance.Red) && redBall1 && ball1Detected)
        ||
        (alliance.equals(Alliance.Blue) && blueBall1 && ball1Detected);
  }

  public boolean getInnerFilled() {
    return ball1Detected;
  }

  public boolean getOuterFilled() {
    return ball0Detected;
  }

  public static ColorSensor getInstance() {
    return _instance;
  }

  @Override
  public void periodic() {
    if (updateCycles % 2 == 0) {
      updateSensor0();
      updateSensor1();

      SmartDashboard.putData("Inner Ball", innerBall);
      SmartDashboard.putData("Outer Ball", outerBall);
    }
    updateCycles++;
    // This method will be called once per scheduler run
  }
}