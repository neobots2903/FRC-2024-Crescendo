package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants.ArmConstants;
 
/*
 * before encoder gearing ratio is 300:1 
 * after encoder chain ratio 5:1
 */

public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax m_armMotor;
  private final DutyCycleEncoder m_armEncoder;

  private final CANSparkMax m_armExtendMotor;
  private final RelativeEncoder m_armExtendEncoder;
  private final SparkPIDController m_extendPid;

  // Normally open limit switch
  private final DigitalInput m_armStopLimit;
  
  private final ArmFeedforward m_feedForward;

  private Boolean is_extended = false;

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kArmP,
            ArmConstants.kArmI,
            ArmConstants.kArmD,
            new TrapezoidProfile.Constraints(
                ArmConstants.kArmMaxVelocityRadPerSecond,
                ArmConstants.kArmMaxAccelerationRadPerSecSquared)),
        0);

    m_armMotor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
    m_armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoderPort);
    m_armStopLimit = new DigitalInput(ArmConstants.kArmStopLimitPort);

    m_armExtendMotor = new CANSparkMax(ArmConstants.kArmExtendMotorPort, MotorType.kBrushless);
    m_armExtendEncoder = m_armExtendMotor.getEncoder();
    m_extendPid = m_armExtendMotor.getPIDController();

    m_feedForward = new ArmFeedforward(ArmConstants.kSArmVolts, ArmConstants.kGArmVolts,
        ArmConstants.kVArmVoltSecondPerRad, ArmConstants.kAArmVoltSecondSquaredPerRad);

    m_armEncoder.setDistancePerRotation(ArmConstants.kArmEncoderDistancePerPulse);

    // Start arm at rest in neutral position
    // setGoal(ArmConstants.kArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedForward.calculate(setpoint.position, setpoint.velocity);

    // m_armMotor.setVoltage(output + feedforward);
    m_armMotor.setVoltage(0);

    // Right place for this???
    // Should be more robust to stop from getting stuck
    if (getLimitSwitch()) {
      m_armMotor.stopMotor();
      return;
    }
  }

  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", getMeasurement());
    SmartDashboard.putNumber("Arm Abs Pos", m_armEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Setpoint", getController().getSetpoint().position);
    SmartDashboard.putNumber("Arm Error", getController().getPositionError());
    SmartDashboard.putNumber("Arm Velocity", m_armEncoder.getFrequency());
    SmartDashboard.putBoolean("Arm Extended", is_extended);
    SmartDashboard.putBoolean("Arm Limit", getLimitSwitch());
  }

  public Boolean isExtended() {
    return is_extended;
  }

  public Boolean getLimitSwitch() {
    return !m_armStopLimit.get();
  }

  public void extendArm() {
    // Run arm extension motor to 12 inches.
    m_extendPid.setReference(ArmConstants.kArmExtendPos, CANSparkMax.ControlType.kPosition);
    is_extended = true;
  }

  public void retractArm() {
    // Run arm extension motor to 0 inches.
    m_extendPid.setReference(ArmConstants.kArmRetractPos, CANSparkMax.ControlType.kPosition);
    is_extended = false;
  }

  @Override
  public double getMeasurement() {
    return m_armEncoder.getDistance() + ArmConstants.kArmOffsetRads;
  }
}