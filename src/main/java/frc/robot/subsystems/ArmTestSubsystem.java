package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmTestSubsystem extends ProfiledPIDSubsystem {
    private final CANSparkMax m_motor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
    private final CANSparkMax M_extendMotor = new CANSparkMax(ArmConstants.kArmExtendMotorPort, MotorType.kBrushless);
    private final RelativeEncoder m_extendEncoder;
    private final SparkPIDController m_extendPid;
    private Boolean is_extended = false;
    private DigitalInput m_stopLimit = new DigitalInput(ArmConstants.kArmStopLimitPort);

    private final DutyCycleEncoder m_encoder =
        new DutyCycleEncoder(ArmConstants.kArmEncoderPort);

    // private final ArmFeedforward m_feedforward =
    //     new ArmFeedforward(
    //         ArmConstants.kSArmVolts, ArmConstants.kGArmVolts,
    //         ArmConstants.kVArmVoltSecondPerRad, ArmConstants.kAArmVoltSecondSquaredPerRad);

    public ArmTestSubsystem() {
        super(
            new ProfiledPIDController(
                ArmConstants.kArmP,
                0,                
                0,
                new TrapezoidProfile.Constraints(
                    ArmConstants.kArmMaxVelocityRadPerSecond,
                    ArmConstants.kArmMaxAccelerationRadPerSecSquared)),
            0);

        // m_encoder.setDistancePerRotation(ArmConstants.kArmEncoderDistancePerPulse);

        m_extendEncoder = M_extendMotor.getEncoder();
        m_extendPid = M_extendMotor.getPIDController();

        m_extendPid.setP(ArmConstants.kArmExtendP);
        m_extendPid.setI(ArmConstants.kArmExtendI);
        m_extendPid.setD(ArmConstants.kArmExtendD);
        m_extendPid.setOutputRange(ArmConstants.kArmExtendMinOutput, ArmConstants.kArmExtendMaxOutput);
    
        // Start arm at rest in neutral position
        setGoal(ArmConstants.kArmOffsetRads);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
    //   // Calculate the feedforward from the sepoint
    //   double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    //   // Add the feedforward to the PID output to get the motor output
    //   m_motor.setVoltage(output + feedforward);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", getMeasurement());
        SmartDashboard.putNumber("Arm Setpoint", getController().getSetpoint().position);
        SmartDashboard.putNumber("Arm Position", m_encoder.getAbsolutePosition());
        SmartDashboard.putBoolean("Arm Limit", !m_stopLimit.get());

        if (!m_stopLimit.get()) {
        // Stop arm from moving
        disable();
        }
    }

    public Boolean isExtended() {
        return is_extended;
    }

    public Boolean isAtLimit() {
        return !m_stopLimit.get();
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
      return m_encoder.getDistance() + ArmConstants.kArmOffsetRads;
    }

    public Command setArmGoalCommand(double kArmOffsetRads) {
        return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
    }
}
