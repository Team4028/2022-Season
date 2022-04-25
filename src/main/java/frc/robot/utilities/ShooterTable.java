// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.Iterator;
import java.util.LinkedList;

import frc.robot.Constants.ShooterConstants;

/**
 * Add your docs here.
 */
public class ShooterTable {

    private static ShooterTable _primary = new ShooterTable(true);
    // private static ShooterTable _secondary = new ShooterTable(false);

    public static ShooterTable getPrimaryTable() {
        return _primary;
    }

    private int _indexCounter;
    private int _currentIndex = 0;
    private ShooterTableEntry ste;

    private LinkedList<ShooterTableEntry> _Table = null;

    private ShooterTable(boolean isPrimary) {
        _Table = isPrimary ? LoadPrimaryTable() : LoadSecondaryTable();
        _currentIndex = 1;

        Iterator<ShooterTableEntry> itr = _Table.iterator();
        while (itr.hasNext()) {
            ste = itr.next();
        }
    }

    public ShooterTableEntry CalcShooterValues(double distanceInFeet) {
        ShooterTableEntry steBelow = null;
        ShooterTableEntry steAbove = null;
        ShooterTableEntry steCurrent = null;

        Iterator<ShooterTableEntry> itr = _Table.iterator();
        while (itr.hasNext()) {
            steCurrent = itr.next();

            if (steCurrent.DistanceInFeet < distanceInFeet) {
                steBelow = steCurrent;
                continue;
            }

            else if (steCurrent.DistanceInFeet == distanceInFeet) {
                steBelow = steCurrent;
                steAbove = steCurrent;
                break;
            }
            // if longer, snapshot Above, stop looping
            else if (steCurrent.DistanceInFeet > distanceInFeet) {
                steAbove = steCurrent;
                break;
            }
        }

        if (steBelow != null && steAbove != null) {
            if (steBelow.DistanceInFeet == steAbove.DistanceInFeet) {
                ste = steAbove;
            } else {
                // find the scale factor which is how far we are between the below & above ste
                double scaleFactor = (distanceInFeet - steBelow.DistanceInFeet)
                        / (steAbove.DistanceInFeet - steBelow.DistanceInFeet);

                // round to int
                double shooterFrontAdj = scaleFactor * (steAbove.ShooterFrontRPM - steBelow.ShooterFrontRPM);
                int shooterFrontCalculatedRPM = steBelow.ShooterFrontRPM + (int) (Math.round(shooterFrontAdj));
                double shooterBackAdj = scaleFactor * (steAbove.ShooterBackRPM - steBelow.ShooterBackRPM);
                int shooterBackCalculatedRPM = steBelow.ShooterBackRPM + (int) (Math.round(shooterBackAdj));
                double kickerAdj = scaleFactor * (steAbove.KickerRPM - steBelow.KickerRPM);
                int kickerCalculatedRPM = steBelow.KickerRPM + (int) (Math.round(kickerAdj));

                double actuatorValue = steBelow.ActuatorVal
                        + (scaleFactor * (steAbove.ActuatorVal - steBelow.ActuatorVal));
                if (actuatorValue > ShooterConstants.kMaxAllowedAngle) {
                    actuatorValue = ShooterConstants.kMaxAllowedAngle;
                }

                // build the return object
                ste = new ShooterTableEntry(_indexCounter++, distanceInFeet, shooterFrontCalculatedRPM,
                        shooterBackCalculatedRPM, kickerCalculatedRPM, actuatorValue, "Calculated value", false);
            }
        } else if (steAbove != null) {
            ste = steAbove;
        } else {
            ste = steBelow;
        }

        return ste;
    }

    public ShooterTableEntry getNextEntry() {
        if (!get_IsAtUpperEntry()) {
            _currentIndex++;
        }

        return _Table.get(_currentIndex);
    }

    public ShooterTableEntry getCurrentEntry() {
        return _Table.get(_currentIndex);
    }

    public ShooterTableEntry getPreviousEntry() {
        if (!get_IsAtLowerEntry()) {
            _currentIndex--;
        }

        return _Table.get(_currentIndex);
    }

    // ============================================================================================
    // properties follow
    // ============================================================================================

    public Boolean get_IsAtUpperEntry() {
        if (_currentIndex == _Table.size() - 1) {
            return true;
        } else {
            return false;
        }
    }

    public Boolean get_IsAtLowerEntry() {
        if (_currentIndex == 0) {
            return true;
        } else {
            return false;
        }
    }

    // ============================================================================================
    // helpers follow
    // ============================================================================================
    // create a linked list
    private LinkedList<ShooterTableEntry> LoadPrimaryTable() {

        LinkedList<ShooterTableEntry> primarytable = new LinkedList<ShooterTableEntry>();

        _indexCounter = 0;

        // ======================================================================================
        // Position feet Stg1
        // ======================================================================================
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 11.5, 2600, .28));
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 13.38, 3110, .28));
        // //30 //2910|33 //34
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 18.5, 3400, .33));
        // //3400|33 //36 //3200|39 //41
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 15.0, 2600, .28));
        // //3907|37 //40 //3707|43 //44
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 17.0, 2910, .28));
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 20.0, 3003, .35)); //
        // 03/10
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 20.0, 2910, .28));

        // primarytable.add(new ShooterTableEntry(_indexCounter++, 27.5, 4228, .38));
        // //41 //4028|44
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 38, 4300, .48));
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 42, 4400, .50));

        // primarytable.add(new ShooterTableEntry(_indexCounter, 9., 4000, 4400, 60., "9
        // foot example shot", false));
        // primarytable.add(new ShooterTableEntry(_indexCounter, 11., 4100, 4500, 60.,
        // "11 foot example shot", false));
        /*
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 10., 1390, 1180, 60.,
         * "Index 10", false));
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 11., 1455, 1215, 60.,
         * "Index 11", false));
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 12., 1510, 1250, 60.,
         * "Index 12", false));
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 13., 1420, 1550, 60.,
         * "Index 13", false));
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 14., 1320, 1750, 60.,
         * "Index 14", false));
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 15., 1220, 1950, 60.,
         * "Index 15", false));
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 16., 1140, 2140, 60.,
         * "Index 16", false));
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 17., 1365, 1180, 60.,
         * "Index 17", false));
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 18., 1365, 1180, 60.,
         * "Index 18", false));
         */

        /*
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 11., 1320, 1740,
         * 110., "Index 11", false));
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 14., 1320, 2340,
         * 140., "Index 14", false));
         * primarytable.add(new ShooterTableEntry(_indexCounter++, 19., 1475, 2575,
         * 190., "Index 19", false));
         */

        // primarytable.add(new ShooterTableEntry(_indexCounter++, 7., 35, 50, 7.0,
        // "index 7", false));
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 11., 45, 45, 14.0,
        // "index 11", false));
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 14., 40, 60, 0.,
        // "index 14", false));
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 19., 40, 80, 0.,
        // "index 19", false));

        if (ShooterConstants.kIsRealGoal) {
            if (ShooterConstants.kIsVBus) {
                primarytable.add(new ShooterTableEntry(_indexCounter++, 7., 40, 50, 55,
                        7., "close lob", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 10., 39, 59, 55,
                        14., "index 10", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 12.6, 41, 63, 55,
                        19., "index 12.5", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 15.1, 43, 69, 55,
                        30., "index 15", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 19.1, 49, 78, 55,
                        30., "index 19", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 22.1, 54, 85, 55,
                        33., "index 22", false));
            } else {
                // TODO: double these values
                primarytable.add(new ShooterTableEntry(_indexCounter++, 7., 1180, 1460, 55, 7., "close lob", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 10., 1130, 1740, 55, 14., "index 10", false));
                primarytable
                        .add(new ShooterTableEntry(_indexCounter++, 12.5, 1260, 2040, 55, 22, "index 12.5", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 15, 1100, 1940, 55, 30., "index 15", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 19, 1440, 2300, 55, 30., "index 19", false));
            }
        } else {
            if (ShooterConstants.kIsVBus) {
                primarytable.add(new ShooterTableEntry(_indexCounter++, 7., 42, 35, 55,
                        9., "close lob", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 10., 33, 48, 55,
                        18., "index 10", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 12.5, 35 + 4, 55 + 4,
                        55, 27.5, "index 12.5", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 15, 38, 64, 55,
                        33., "index 15", false));
            } else {
                // TODO: double these values
                primarytable.add(new ShooterTableEntry(_indexCounter++, 7., 1240, 980, 55, 9., "close lob", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 10., 950, 1400, 55, 18., "index 10", false));
                primarytable
                        .add(new ShooterTableEntry(_indexCounter++, 12.5, 1000, 1620, 55, 24.5, "index 12.5", false));
                primarytable.add(new ShooterTableEntry(_indexCounter++, 15, 1100, 1940, 55, 30., "index 15", false));

            }
        }

        return primarytable;
    }

    private LinkedList<ShooterTableEntry> LoadSecondaryTable() {

        LinkedList<ShooterTableEntry> secondarytable = new LinkedList<ShooterTableEntry>();

        _indexCounter = 0;
        // ======================================================================================
        // Position feet Stg1
        // ======================================================================================
        // secondarytable.add(new ShooterTableEntry(_indexCounter++, 25, 2910, .33));
        // secondarytable.add(new ShooterTableEntry(_indexCounter++, 27, 2910, .33));
        // secondarytable.add(new ShooterTableEntry(_indexCounter++, 29, 2910, .33));
        return secondarytable;
    }
}