package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmTestSubsystem extends SubsystemBase {
    private final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushed); //needs to be changed for brushless motors
    
    private SparkPIDController m_pidController = m_armMotor.getPIDController();
    private final DutyCycleEncoder m_encoder =
        new DutyCycleEncoder(ArmConstants.kArmEncoderPort);
    
    private double targetDegrees = 0;

    public ArmTestSubsystem() {
        // set PID coefficients
        m_pidController.setP(ArmConstants.kArmP);
        m_pidController.setI(ArmConstants.kArmI);
        m_pidController.setD(ArmConstants.kArmD);
        m_pidController.setOutputRange(ArmConstants.kArmMaxVelocityRadPerSecond,
                ArmConstants.kArmMaxAccelerationRadPerSecSquared);
        // make encoder use degrees instead of rotations
        // m_encoder.setPositionConversionFactor(360.0 / 3.0);

        // Put PID values in SmartDashboard for tuning
        SmartDashboard.putNumber("P Gain", ArmConstants.kArmP);
        SmartDashboard.putNumber("I Gain", ArmConstants.kArmI);
        SmartDashboard.putNumber("D Gain", ArmConstants.kArmD);
        SmartDashboard.putNumber("Max Output", ArmConstants.kArmMaxVelocityRadPerSecond);
        SmartDashboard.putNumber("Min Output", ArmConstants.kArmMaxAccelerationRadPerSecSquared);
        SmartDashboard.putNumber("Arm Target Angle", 0);
    }

    private void UpdatePIDCoefficients() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // // if PID coefficients on SmartDashboard have changed, write new values to controller
        // if((p != ArmConstants.kP)) { m_pidController.setP(p); ArmConstants.kP = p; }
        // if((i != ArmConstants.kI)) { m_pidController.setI(i); ArmConstants.kI = i; }
        // if((d != ArmConstants.kD)) { m_pidController.setD(d); ArmConstants.kD = d; }
        // if((iz != ArmConstants.kIz)) { m_pidController.setIZone(iz); ArmConstants.kIz = iz; }
        // if((ff != ArmConstants.kFF)) { m_pidController.setFF(ff); ArmConstants.kFF = ff; }
        // if((max != ArmConstants.kMaxOutput) || (min != ArmConstants.kMinOutput)) { 
        //     m_pidController.setOutputRange(min, max); 
        //     ArmConstants.kMinOutput = min; ArmConstants.kMaxOutput = max; 
        // }
    }

    public double getDegrees() {
        return m_encoder.getAbsolutePosition();
    }

    public void zeroEncoder() {
        m_encoder.reset();
    }

    public void goToDegrees(double degrees) {
        targetDegrees = degrees;
        m_pidController.setReference(degrees, CANSparkBase.ControlType.kPosition);
        SmartDashboard.putNumber("Arm Target Angle", degrees);
    }

    public enum ArmPosition {
        Initial,
        Pickup,
        Amp,
        Speaker
    }

    public Command setArmPosition(ArmPosition pos) {
        return new InstantCommand(() -> {
            double armAngle;
            switch (pos) {
            case Initial:
                armAngle = ArmConstants.kArmRestingPosition;
                break;
            case Pickup:
                armAngle = ArmConstants.kArmIntakePosition;
                break;
            case Amp:
                armAngle = ArmConstants.kArmAmpPosition;
                break;
            case Speaker:
                armAngle = ArmConstants.kArmSpeakerPosition;
                break;
            default:
                // Don't do anything
                return;
            }
            goToDegrees(armAngle);
        }, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Current Angle", getDegrees());
        SmartDashboard.putNumber("Motor power", m_armMotor.getOutputCurrent());

        double newTargetDegrees = SmartDashboard.getNumber("Arm Target Angle", 0);
        if (targetDegrees != newTargetDegrees) { goToDegrees(newTargetDegrees); }
        
        UpdatePIDCoefficients();
    }
}
