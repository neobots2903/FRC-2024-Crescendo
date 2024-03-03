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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
 
/*
 * before encoder gearing ratio is 300:1 
 * after encoder chain ratio 5:1
 */

public class ArmSubsystemSimple extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  // private final DutyCycleEncoder m_armEncoder;
  private final RelativeEncoder m_armEncoder;
  private final SparkPIDController m_armPid;

  private final CANSparkMax m_armExtendMotor;
  private final RelativeEncoder m_armExtendEncoder;
  private final SparkPIDController m_extendPid;

  // Normally open limit switch
  private final DigitalInput m_armStopLimit;
  
  // private final ArmFeedforward m_feedForward;

  private Boolean is_extended = false;

  /** Create a new ArmSubsystem. */
  public ArmSubsystemSimple() {
    m_armMotor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
    // m_armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoderPort);
    m_armEncoder = m_armMotor.getEncoder();
    m_armStopLimit = new DigitalInput(ArmConstants.kArmStopLimitPort);
    m_armPid = m_armMotor.getPIDController();

    m_armExtendMotor = new CANSparkMax(ArmConstants.kArmExtendMotorPort, MotorType.kBrushless);
    m_armExtendEncoder = m_armExtendMotor.getEncoder();
    m_extendPid = m_armExtendMotor.getPIDController();

    // Set pid coefficients
    m_armPid.setP(ArmConstants.kArmP);
    m_armPid.setI(ArmConstants.kArmI);
    m_armPid.setD(ArmConstants.kArmD);
    m_armPid.setOutputRange(ArmConstants.kArmMinOutput, ArmConstants.kArmMaxOutput);

    m_extendPid.setP(ArmConstants.kArmExtendP);
    m_extendPid.setI(ArmConstants.kArmExtendI);
    m_extendPid.setD(ArmConstants.kArmExtendD);

    // m_feedForward = new ArmFeedforward(ArmConstants.kSArmVolts, ArmConstants.kGArmVolts,
    //     ArmConstants.kVArmVoltSecondPerRad, ArmConstants.kAArmVoltSecondSquaredPerRad);

    m_armEncoder.setPositionConversionFactor(ArmConstants.kArmConversionFactor);
  }

  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", m_armEncoder.getPosition());
    SmartDashboard.putBoolean("Arm Extended", is_extended);
    SmartDashboard.putBoolean("Arm Limit", getLimitSwitch());
    SmartDashboard.putNumber("Arm Amps", m_armMotor.getOutputCurrent());

    if (getLimitSwitch()) {
      m_armMotor.stopMotor();
      return;
    }
  }

  public void goToPosition(double position) {
    SmartDashboard.putNumber("Arm Target", position);
    SmartDashboard.putNumber("Arm Target Converted", position * ArmConstants.kArmConversionFactor);
    m_armPid.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public Boolean isExtended() {
    return is_extended;
  }

  public Boolean getLimitSwitch() {
    return !m_armStopLimit.get();
  }

  public void disableArm() {
    m_armPid.setReference(0, CANSparkMax.ControlType.kDutyCycle);
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