package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
 
/*
 * before encoder gearing ratio is 300:1 
 * after encoder chain ratio 5:1
 */

public class ArmSubsystemSimple extends SubsystemBase {
  // Arm motor and encoder for controlling the pivot rotation.
  private final CANSparkMax m_armMotor;
  private final DutyCycleEncoder m_armEncoder;
  // Normally open limit switch
  private final DigitalInput m_armStopLimit;

  // Pid controller
  private final PIDController m_pidController;

  private Boolean is_extended = false;

  /** Create a new ArmSubsystem. */
  public ArmSubsystemSimple() {
    m_armMotor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
    m_armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoderPort);
    m_armStopLimit = new DigitalInput(ArmConstants.kArmStopLimitPort);

    m_pidController = new PIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD);

    m_armEncoder.setDistancePerRotation(ArmConstants.kArmEncoderDistancePerPulse);
  }

  // Go to a given angle.
  public void goToPosition(double position) {
    m_pidController.setSetpoint(position);
    m_armMotor.set(m_pidController.calculate(m_armEncoder.get()));
  }

  public void disableArm() {
    m_armMotor.set(0);
    m_armMotor.disable();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", m_armEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Velocity", m_armEncoder.getFrequency());
    SmartDashboard.putBoolean("Arm Extended", is_extended);
    SmartDashboard.putBoolean("Arm Limit", getLimitSwitch());

    if (isLimitSwitchPressed()) {
      m_armMotor.set(0);
      // Add logic to allow moving down.
    }
  }

  public boolean isLimitSwitchPressed() {
    return !m_armStopLimit.get();
  }

  public Boolean isExtended() {
    return is_extended;
  }

  public Boolean getLimitSwitch() {
    return !m_armStopLimit.get();
  }
}