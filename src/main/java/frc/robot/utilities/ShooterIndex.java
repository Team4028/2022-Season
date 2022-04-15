// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
/** Add your docs here. */
public class ShooterIndex implements Sendable {
    private double index;
    private double fontSize;
    public ShooterIndex() {}

    public void setIndex(double index) {
        this.index = index;
    }

    public double getIndex() {
        return index;
    }

    public void setFontSize(double size) {
        this.fontSize = size;
    }

    public double getFontSize() {
        return fontSize;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ShooterIndex");
        builder.addDoubleProperty("index", () -> index, null);
        builder.addDoubleProperty("fontSize", () -> fontSize, null);
    }
}
