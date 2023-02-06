// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static final int FRONT_LEFT_ID = 1;
    public static final int FRONT_RIGHT_ID = 2;
    public static final int BACK_LEFT_ID = 3;
    public static final int BACK_RIGHT_ID = 4;
    public static final int PIGEON_ID = 5;

    public static final double DRIVETRAIN_kS = 0.13305;
    public static final double DRIVETRAIN_kV = 2.2876;
    public static final double DRIVETRAIN_kA = 0.31596;

    public static final double[] DRIVETRAIN_CHARACTERIZATION = {DRIVETRAIN_kS, DRIVETRAIN_kV, DRIVETRAIN_kA};

    public static final double kFalconToMeters = (1.0/2048) * (Units.inchesToMeters(6) * Math.PI) * (1/10.71) ; //10.71:1 gearbox

}
