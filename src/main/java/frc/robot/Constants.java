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

    public static final double MIN_SPEED = 0.25;
    public static final double MID_SPEED = 0.626;
    public static final double MAX_SPEED = 1.0;
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
    public static final int kShooterMotorPort = 13;
    public static final double kShooterSpeed  = 0.6;
    public static final int kShooterRPM = 2500;

    public static final int kIntakeMotorPort = 11;
    public static final double kIntakeSpeed = 0.75;
    public static final double kIntakeShootSpeed = 1;
    public static final int kIntakeLimitPort = 2; // FIND REAL VALUE
  }

  public static class ArmConstants
  {
    public static final int kArmMotorPort = 10;
    public static final int kArmExtendMotorPort = 12;
    public static final int kArmStopLimitPort = 0;
    public static final int kArmEncoderPort = 1;

    // Rotations of the motor to get to the desired position
    public static final int kArmExtendPos = 70;
    public static final int kArmRetractPos = 0;

    // Arm Rotation PID
    public static final double kArmP = 0.1;
    public static final double kArmI = 0;
    public static final double kArmD = 0.005;
    public static final double kArmMaxOutput = 0.6;
    public static final double kArmMinOutput = -0.2;
    public static final double kArmMaxVelocityRadPerSecond = 3;
    public static final double kArmMaxAccelerationRadPerSecSquared = 10;

    // Arm Extension PID
    public static final double kArmExtendP = 0.02;
    public static final double kArmExtendI = 0;
    public static final double kArmExtendD = 0;
    public static final double kArmExtendMaxOutput = 0.8;
    public static final double kArmExtendMinOutput = -0.8;

    // 300:1 before encoder, 5:1 after encoder
    public static final double kArmConversionFactor = 1.19;

    // Angle above the bumpers
    public static final double kArmBumperPos = 20;

    // Arm positions degrees.
    public static final double kArmRestingPosition = 0;
    public static final double kArmIntakePosition = -5;
    public static final double kArmSpeakerPosition = 5;
    public static final double kArmAmpPosition = 90;
  }
}