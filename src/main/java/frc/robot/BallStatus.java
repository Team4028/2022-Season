// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.shuffleboard.api.data.ComplexData;

/** Add your docs here. */
public class BallStatus extends ComplexData<BallStatus> {
    private final boolean hasRed;
    private final boolean hasBlue;
    private final boolean hasBall;

    // Constructor should take all the different fields needed and assign them their
    // corresponding instance variables.
    public BallStatus(boolean hasRed, boolean hasBlue, boolean hasBall) {
        this.hasRed = hasRed;
        this.hasBlue = hasBlue;
        this.hasBall = hasBall;
    }

    public BallStatus setRed(boolean has) {
        return new BallStatus(has, hasBlue, hasBall);
    }

    public BallStatus setBlue(boolean has) {
        return new BallStatus(hasRed, has, hasBall);
    }

    public BallStatus setBall(boolean has) {
        return new BallStatus(hasRed, hasBlue, has);
    }

    @Override
    public Map<String, Object> asMap() {
        return Map.of("hasRed", hasRed, "hasBlue", hasBlue, "hasBall", hasBall);
    }
}
