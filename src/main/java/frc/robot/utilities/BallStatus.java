// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
/** Add your docs here. */
public class BallStatus implements Sendable {
    private boolean hasRed, hasBlue, hasBall;
    public BallStatus() {}

    public void setRed(boolean has) {
        hasRed = has;
    }

    public void setBlue(boolean has) {
        hasBlue = has;
    }

    public void setBall(boolean has) {
        hasBall = has;
    }

    public boolean getRed() {
        return hasRed;
    }

    public boolean getBlue() {
        return hasBlue;
    }

    public boolean getBall() {
        return hasBall;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Point2D");
        builder.addBooleanProperty("hasRed", () -> hasRed, null);
        builder.addBooleanProperty("hasBlue", () -> hasBlue, null);
        builder.addBooleanProperty("hasBall", () -> hasBall, null);
    }
}
