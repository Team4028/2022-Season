// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

public class ShooterTableEntry {
    public final int Index;
    public final double DistanceInFeet;
    public final int ShooterFrontRPM;
    public final int ShooterBackRPM;
    public final int KickerRPM;
    public final double ActuatorVal;
    public final double ShotTime;
    public final String Description;
    public final boolean IsDefault;

    // ============================================================================================
    // constructors follow
    // ============================================================================================
    public ShooterTableEntry(
            int index,
            double distanceInFeet,
            int shooterFrontRPM,
            int shooterBackRPM,
            int kickerRPM,
            double actuatorValue,
            double shotTime,
            String description,
            boolean isDefault) {
        Index = index;
        DistanceInFeet = distanceInFeet;
        ShooterFrontRPM = shooterFrontRPM;
        ShooterBackRPM = shooterBackRPM;
        KickerRPM = kickerRPM;
        ActuatorVal = actuatorValue;
        ShotTime = shotTime;
        Description = description;
        IsDefault = isDefault;
    }

}