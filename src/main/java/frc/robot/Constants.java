// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

/** Add your docs here. */
public final class Constants {

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.0;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.0;

    public static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(
        // Front left
        new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
            Constants.DRIVETRAIN_WHEELBASE_METERS),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
            -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
            Constants.DRIVETRAIN_WHEELBASE_METERS),
        // Back right
        new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
            -Constants.DRIVETRAIN_WHEELBASE_METERS));

    public static final int PRIMARYCONTROLLERPORT = 0;

    public static final int frontLeftAngle = 225;
    public static final int frontRightAngle = 135;
    public static final int backLeftAngle = 315;
    public static final int backRightAngle = 45;

}
