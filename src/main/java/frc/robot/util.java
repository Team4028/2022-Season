// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class util {
    public static double toFalconRPM(double velocity) {
        return velocity * 600 / 4096;
    }

    public static double toFalconVelocity(double rpm) {
        return rpm * 4096 / 600;
    }
    public static double deadband(double input){
        return Math.abs(input) > 0.025? input: 0.0;
    }
}
