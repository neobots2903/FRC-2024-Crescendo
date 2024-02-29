package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants.ArmConstants;
 
/*
 * complete arm ratio is 75:1 
 * encoder ratio 5:1
 */

public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final PWMSparkMax m_motor = new PWMSparkMax(ArmConstants.kArmMotorPort);
  private final Encoder m_encoder =
      new Encoder(ArmConstants.kArmEncoderPorts[0], ArmConstants.kArmEncoderPorts[1]);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSArmVolts, ArmConstants.kGArmVolts,
          ArmConstants.kVArmVoltSecondPerRad, ArmConstants.kAArmVoltSecondSquaredPerRad);

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
    m_encoder.setDistancePerPulse(ArmConstants.kArmEncoderDistancePerPulse);
    // Start arm at rest in neutral position
    setGoal(ArmConstants.kArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motor.setVoltage(output + feedforward);
  }

  public Boolean isExtended() {
    return is_extended;
  }

  @Override
  public double getMeasurement() {
    return m_encoder.getDistance() + ArmConstants.kArmOffsetRads;
  }
}