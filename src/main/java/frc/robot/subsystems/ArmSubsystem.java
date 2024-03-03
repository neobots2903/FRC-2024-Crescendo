package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
 
/*
 * complete arm ratio is 300:1 
 * encoder ratio 5:1
 */

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
  private final CANSparkMax M_extendMotor = new CANSparkMax(ArmConstants.kArmExtendMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_extendEncoder;
  private final RelativeEncoder m_armEncoder;
  private final SparkPIDController m_extendPid, m_armPid;
  public double kMaxOutput, kMinOutput, maxRPM, maxVel, maxAcc;

  // private final DutyCycleEncoder m_encoder =
  //     new DutyCycleEncoder(ArmConstants.kArmEncoderPort);

  // private final ArmFeedforward m_feedforward =
  //     new ArmFeedforward(
  //         ArmConstants.kSArmVolts, ArmConstants.kGArmVolts,
  //         ArmConstants.kVArmVoltSecondPerRad, ArmConstants.kAArmVoltSecondSquaredPerRad);

  private DigitalInput m_stopLimit = new DigitalInput(ArmConstants.kArmStopLimitPort);
  private Boolean is_extended = false;

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    // m_encoder.setDistancePerPulse(ArmConstants.kArmEncoderDistancePerPulse);
    // m_encoder.setDistancePerRotation(ArmConstants.kArmEncoderDistancePerPulse);

    m_extendEncoder = M_extendMotor.getEncoder();
    m_extendPid = M_extendMotor.getPIDController();

    m_armEncoder = m_motor.getEncoder();
    m_armPid = m_motor.getPIDController();

    // m_armEncoder.setPositionConversionFactor(300);

    // m_encoder.reset();

    m_extendPid.setP(ArmConstants.kArmExtendP);
    m_extendPid.setI(ArmConstants.kArmExtendI);
    m_extendPid.setD(ArmConstants.kArmExtendD);
    m_extendPid.setOutputRange(ArmConstants.kArmExtendMinOutput, ArmConstants.kArmExtendMaxOutput);

    m_armPid.setP(ArmConstants.kArmP);
    m_armPid.setI(ArmConstants.kArmI);
    m_armPid.setD(ArmConstants.kArmD);
    m_armPid.setOutputRange(-0.5, 0.5);

    // Smart Motion Coefficients
    maxVel = 3000; // rpm
    maxAcc = 2000;

    int smartMotionSlot = 0;
    m_armPid.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_armPid.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Position", m_armEncoder.getPosition());
    SmartDashboard.putNumber("Arm Target", m_armPid.getOutputMax());
    SmartDashboard.putBoolean("Arm Limit", !m_stopLimit.get());

    if (!m_stopLimit.get()) {
      // Stop arm from moving
      
    }
  }

  public Boolean isExtended() {
    return is_extended;
  }

  public void setArmPosition(double degrees) {
    m_armPid.setReference(degrees, CANSparkMax.ControlType.kSmartMotion);
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
}