// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double LBS_TO_KG = 0.453592;
  public static final double ROBOT_MASS = 60 * LBS_TO_KG; // 80lbs * kg per lb, orig: (148 - 20.3) * 0.453592;
  public static final double INTAKE_MASS = 25 * LBS_TO_KG; // 20lbs * kg per lb
  public static final Matter INTAKE = new Matter(new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(5)), INTAKE_MASS);
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(2)), ROBOT_MASS); // orig: 8
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity 
  
  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final int   DRIVER_CONTROLLER_PORT = 0;
    public static final int   OPERATOR_CONTROLLER_PORT = 1;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class IntakeShooterConstants
  {
    public static final int kShooterMotorPort = 1; // TBD
    public static final double kShooterSpeed  = 0.75;
    public static final int kShooterRPM = 5000;

    public static final int kIntakeMotorPort     = 5; // TBD
    public static final double kIntakeSpeed      = 0.85;
    public static final double kIntakeShootSpeed = 1;
  }

  public static class ArmConstants
  {
    public static final int kArmMotorPort = 4;

    // Double check these first!
    public static final double kArmP = 6;
    public static final double kArmI = 0.5;
    public static final double kArmD = 0.75;

    // STarting with estimated values from calculator
    public static final double kSArmVolts = 1; // ???
    public static final double kGArmVolts = 0.72;
    public static final double kVArmVoltSecondPerRad = 5.85;
    public static final double kAArmVoltSecondSquaredPerRad = 0.11;

    public static final double kArmMaxVelocityRadPerSecond = 3;
    public static final double kArmMaxAccelerationRadPerSecSquared = 10;

    public static final int[] kArmEncoderPorts = new int[] {4, 5};
    public static final int kArmEncoderPPR = 256;
    public static final double kArmEncoderDistancePerPulse = 2.0 * Math.PI / kArmEncoderPPR;

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kArmOffsetRads = 5;

    // NEED TO FIND REAL NUMBERS!!! (Radians?)
    public static final double kArmIntakePosition = kArmOffsetRads;
    public static final double kArmAmpPosition = 40;
    public static final double kArmSpeakerPosition = 90;
  }
}