package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
 
/*
 * before encoder gearing ratio is 300:1 
 * after encoder chain ratio 5:1
 */

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final RelativeEncoder m_armEncoder;
  private final SparkPIDController m_armPid;

  private final CANSparkMax m_armExtendMotor;
  private final RelativeEncoder m_armExtendEncoder;
  private final SparkPIDController m_extendPid;

  // Normally open limit switch
  private final DigitalInput m_armStopLimit;

  // local variables
  private Boolean is_extended = false;
  private Boolean lastSwitchState = false;
  private double currentTarget = 0;

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
    m_armEncoder = m_armMotor.getEncoder();
    m_armStopLimit = new DigitalInput(ArmConstants.kArmStopLimitPort);
    m_armPid = m_armMotor.getPIDController();
    m_armEncoder.setPosition(0);

    m_armExtendMotor = new CANSparkMax(ArmConstants.kArmExtendMotorPort, MotorType.kBrushless);
    m_armExtendEncoder = m_armExtendMotor.getEncoder();
    m_extendPid = m_armExtendMotor.getPIDController();
    m_armExtendEncoder.setPosition(0);

    // Set pid coefficients
    m_armPid.setP(ArmConstants.kArmP);
    m_armPid.setI(ArmConstants.kArmI);
    m_armPid.setD(ArmConstants.kArmD);
    m_armPid.setFF(ArmConstants.kArmFF);
    m_armPid.setOutputRange(ArmConstants.kArmMinOutput, ArmConstants.kArmMaxOutput);

    m_extendPid.setP(ArmConstants.kArmExtendP);
    m_extendPid.setI(ArmConstants.kArmExtendI);
    m_extendPid.setD(ArmConstants.kArmExtendD);
    m_extendPid.setOutputRange(ArmConstants.kArmExtendMinOutput, ArmConstants.kArmExtendMaxOutput);

    m_armEncoder.setPositionConversionFactor(ArmConstants.kArmConversionFactor);
  }

  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", m_armEncoder.getPosition());
    SmartDashboard.putBoolean("Arm Extended", is_extended);
    SmartDashboard.putBoolean("Arm Limit", getLimitSwitch());
    SmartDashboard.putNumber("Arm Amps", m_armMotor.getOutputCurrent());
    
    SmartDashboard.putNumber("Arm Ext Amps", m_armExtendMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Above Bumpers", isArmAboveBumpers());

    // Allow it to go down because the switch limits up only.
    // MODIFY CODE, KINDA WORKS (have to tap down multiple times)
    if (getLimitSwitch()) {
      if (lastSwitchState) {
        return;
      }
      m_armMotor.stopMotor();
      m_armPid.setOutputRange(ArmConstants.kArmExtendMinOutput, 0);
      lastSwitchState = true;
      return;
    } else {
      if (!lastSwitchState) {
        return;
      }
      m_armPid.setOutputRange(ArmConstants.kArmMinOutput, ArmConstants.kArmMaxOutput);
      lastSwitchState = false;
    }
  }

  public void goToPosition(double position) {
    currentTarget = position;
    SmartDashboard.putNumber("Arm Target", currentTarget);
    SmartDashboard.putNumber("Arm Target Converted", currentTarget * ArmConstants.kArmConversionFactor);
    m_armPid.setReference(currentTarget, CANSparkMax.ControlType.kPosition);
  }

  public void decrementPosition() {
    currentTarget -= 5;
    m_armPid.setReference(currentTarget, CANSparkMax.ControlType.kPosition);
  }

  public void incrementPosition() {
    currentTarget += 5;
    m_armPid.setReference(currentTarget, CANSparkMax.ControlType.kPosition);   
  }

  public void zeroArmEncoder() {
    m_armEncoder.setPosition(0);
  }

  public Boolean isExtended() {
    return is_extended && m_armEncoder.getPosition() >= 5;
  }

  public Boolean isRetracted() {
    return !is_extended && m_armEncoder.getPosition() <= 5;
  }

  public Boolean isArmAboveBumpers() {
    return m_armEncoder.getPosition() >= ArmConstants.kArmBumperPos;
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

  public void setCoastMode(boolean coast) {
    m_armMotor.setIdleMode(coast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
    m_armExtendMotor.setIdleMode(coast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
  }
}