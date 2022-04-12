// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PicoColorSensor;
import frc.robot.PicoColorSensor.RawColor;

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

  private boolean ball0Detected;
  private boolean ball1Detected;

  private static ColorSensor _instance = new ColorSensor();

  /** Creates a new ColorSensor. */
  public ColorSensor() {}

  public void updateSensor0(){
    RawColor rc0 = m_sensor.getRawColor0();

    int red0 = rc0.red;
    int green0 = rc0.green;
    int blue0 = rc0.blue;
    int proximity0 = m_sensor.getProximity0();

    int higher = Math.max(red0, blue0);

    if (proximity0 > proximityThreshold0) {
      if (red0 > redThreshold0 && higher == red0) {
        SmartDashboard.putBoolean("red ball 0", true);
        if (!redBallFoundLastCycle) {
          redBalls++;
        }
        redBallFoundLastCycle = true;
      }
      if (blue0 > blueThreshold0 && higher == blue0) {
        SmartDashboard.putBoolean("blue ball 0", true);
        if (!blueBallFoundLastCycle) {
          blueBalls++;
        }
        blueBallFoundLastCycle = true;
      }
    } else {
      redBallFoundLastCycle = false;
      blueBallFoundLastCycle = false;
      SmartDashboard.putBoolean("red ball 0", false);
      SmartDashboard.putBoolean("blue ball 0", false);
    }

    SmartDashboard.putNumber("red 0", red0);
    SmartDashboard.putNumber("blue 0", blue0);
    SmartDashboard.putNumber("proximity 0", proximity0);
    SmartDashboard.putNumber("total red balls", redBalls);
    SmartDashboard.putNumber("total blue balls", blueBalls);
    ball0Detected = proximity0 > proximityThreshold0;

  }

  public void updateSensor1(){
    RawColor rc1 = m_sensor.getRawColor1();

    int red1 = rc1.red;
    int green1 = rc1.green;
    int blue1 = rc1.blue;
    int proximity1 = m_sensor.getProximity1();

    int higher = Math.max(red1, blue1);

    if (proximity1 > proximityThreshold1) {
      SmartDashboard.putBoolean("red ball 1", red1 > redThreshold1 && higher == red1);
      SmartDashboard.putBoolean("blue ball 1", blue1 > blueThreshold1 && higher == blue1);
    } else {
      SmartDashboard.putBoolean("red ball 1", false);
      SmartDashboard.putBoolean("blue ball 1", false);
    }

    SmartDashboard.putNumber("red 1", red1);
    SmartDashboard.putNumber("blue 1", blue1);
    SmartDashboard.putNumber("proximity 1", proximity1);
    ball1Detected = proximity1 > proximityThreshold1;
  }

public static ColorSensor getInstance(){
  return _instance;
}



  @Override
  public void periodic() {
    updateSensor0();
    updateSensor1();
    // This method will be called once per scheduler run
  }
}
