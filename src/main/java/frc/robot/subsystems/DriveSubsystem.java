// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.NoULib.lib.NoUMotor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private static final NoUMotor frontLeft = new NoUMotor(2);
  private static final NoUMotor frontRight = new NoUMotor(3);
  private static final NoUMotor backLeft = new NoUMotor(1);
  private static final NoUMotor backRight = new NoUMotor(5);

  private final MecanumDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    frontLeft.setInverted(false);
    frontRight.setInverted(true);
    backLeft.setInverted(false);
    backRight.setInverted(false);

    m_odometry = new MecanumDriveOdometry(Constants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(0.0), 
      getWheelPositions());
  }

  public void drive2(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(45.0));
    MecanumDriveWheelSpeeds wheelSpeeds = Constants.DRIVE_KINEMATICS.toWheelSpeeds(speeds);

    frontLeft.set(wheelSpeeds.frontLeftMetersPerSecond);
    frontRight.set(wheelSpeeds.frontRightMetersPerSecond);
    backLeft.set(wheelSpeeds.rearLeftMetersPerSecond);
    backRight.set(wheelSpeeds.rearRightMetersPerSecond);
  }

  public void updateOdometry() {
    m_odometry.update(Rotation2d.fromDegrees(0.0), getWheelPositions());
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(frontLeft.get(), frontRight.get(), backLeft.get(), backRight.get());
  }

  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldOrient) {
    double driveAngle = 0.0;

    double frontLeftSpeed = 0.0;
    double frontRightSpeed = 0.0;
    double backLeftSpeed = 0.0;
    double backRightSpeed = 0.0;

    double yawAngle = 0.0;

    double throttle = Math.sqrt((xSpeed * xSpeed) + (ySpeed * ySpeed));
    if (throttle > 1) {
      throttle = 1;
    }

    driveAngle = Math.atan2(xSpeed, ySpeed) * (180 / Math.PI) - 90;

    if (!fieldOrient) {
      frontLeftSpeed = Math.cos((Constants.frontLeftAngle - driveAngle) * 0.01745329251) * throttle;
      frontRightSpeed = Math.cos((Constants.frontRightAngle - driveAngle) * 0.01745329251) * throttle;
      backLeftSpeed = Math.cos((Constants.backLeftAngle - driveAngle) * 0.01745329251) * throttle;
      backRightSpeed = Math.cos((Constants.backRightAngle - driveAngle) * 0.01745329251) * throttle;

      frontLeftSpeed += rotation;
      frontRightSpeed += rotation;
      backLeftSpeed += rotation;
      backRightSpeed += rotation;

    }
    if (fieldOrient) {
      double rotX = xSpeed * Math.cos(-yawAngle) - ySpeed * Math.sin(-yawAngle);
      double rotY = xSpeed * Math.sin(-yawAngle) + ySpeed * Math.cos(-yawAngle);
      double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1.0);

      frontLeftSpeed = (rotY + rotX + rotation) / denominator;
      frontRightSpeed = (rotY - rotX + rotation) / denominator;
      backLeftSpeed = (rotY - rotX - rotation) / denominator;
      backRightSpeed = (rotY + rotX - rotation) / denominator;
    }

    double highestValue = 0.0;
    int highestMotor = 0;
    if (Math.abs(frontLeftSpeed) > highestValue) {
      highestValue = Math.abs(frontLeftSpeed);
      highestMotor = 1;
    }
    if (Math.abs(frontRightSpeed) > highestValue) {
      highestValue = Math.abs(frontRightSpeed);
      highestMotor = 2;
    }
    if (Math.abs(backLeftSpeed) > highestValue) {
      highestValue = Math.abs(backLeftSpeed);
      highestMotor = 3;
    }
    if (Math.abs(backRightSpeed) > highestValue) {
      highestValue = Math.abs(backRightSpeed);
      highestMotor = 4;
    }

    if (highestValue > 1) {
      double amountOver = 0.0;
      switch (highestMotor) {
        case 1:
          amountOver = Math.abs(frontLeftSpeed) - 1;
          frontLeftSpeed = Math.signum(frontLeftSpeed);

          frontRightSpeed -= amountOver * Math.signum(frontRightSpeed);
          backLeftSpeed -= amountOver * Math.signum(backLeftSpeed);
          backRightSpeed -= amountOver * Math.signum(backRightSpeed);
          break;

        case 2:
          amountOver = Math.abs(frontRightSpeed) - 1;
          frontRightSpeed = Math.signum(frontRightSpeed);

          frontLeftSpeed -= amountOver * Math.signum(frontLeftSpeed);
          backLeftSpeed -= amountOver * Math.signum(backLeftSpeed);
          backRightSpeed -= amountOver * Math.signum(backRightSpeed);
          break;

        case 3:
          amountOver = Math.abs(backLeftSpeed) - 1;
          backLeftSpeed = Math.signum(backLeftSpeed);

          frontRightSpeed -= amountOver * Math.signum(frontRightSpeed);
          frontLeftSpeed -= amountOver * Math.signum(frontLeftSpeed);
          backRightSpeed -= amountOver * Math.signum(backRightSpeed);
          break;

        case 4:
          amountOver = Math.abs(backRightSpeed) - 1;
          backRightSpeed = Math.signum(backRightSpeed);

          frontRightSpeed -= amountOver * Math.signum(frontRightSpeed);
          backLeftSpeed -= amountOver * Math.signum(backLeftSpeed);
          frontLeftSpeed -= amountOver * Math.signum(frontLeftSpeed);
          break;
      }
    }

    double fls = -Math.sqrt(Math.abs(frontLeftSpeed)) * Math.signum(frontLeftSpeed);
    double frs = -Math.sqrt(Math.abs(frontRightSpeed)) * Math.signum(frontRightSpeed);
    double bls = -Math.sqrt(Math.abs(backLeftSpeed)) * Math.signum(backLeftSpeed);
    double brs = -Math.sqrt(Math.abs(backRightSpeed)) * Math.signum(backRightSpeed);

    frontLeft.set(fls);
    frontRight.set(frs);
    backLeft.set(bls);
    backRight.set(brs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
