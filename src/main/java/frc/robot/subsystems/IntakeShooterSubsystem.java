package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeShooterConstants;

public class IntakeShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_shooter = new CANSparkMax(IntakeShooterConstants.kShooterMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_intake = new CANSparkMax(IntakeShooterConstants.kIntakeMotorPort, MotorType.kBrushless);

  public enum IntakeDirection {
    IN, OUT
  }

  public IntakeShooterSubsystem() {
    m_shooter.setInverted(false);
    m_intake.setInverted(true);
    m_intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_shooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void startShooter(double speed) {
    m_shooter.set(speed);
  }

  public void startShooter() {
    m_shooter.set(IntakeShooterConstants.kShooterSpeed);
  }

  public void stopShooter() {
    m_shooter.set(0);
  }

  public void startIntakeShoot() {
    m_intake.set(IntakeShooterConstants.kIntakeShootSpeed);
  }

  public void startIntake(IntakeDirection direction, double speed) {
    if (direction == IntakeDirection.IN) {
      m_intake.set(speed);
    } else {
      m_intake.set(-speed);
    }
  }

  public void startIntake(IntakeDirection direction) {
    if (direction == IntakeDirection.IN) {
      m_intake.set(IntakeShooterConstants.kIntakeSpeed);
    } else {
      m_intake.set(-IntakeShooterConstants.kIntakeSpeed);
    }
  }

  public void stopIntake() {
    m_intake.set(0);
  }

  public double getShooterSpeed() {
    return m_shooter.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());
    SmartDashboard.putNumber("Intake Speed", Math.abs(m_intake.getEncoder().getVelocity()));
  }
}
