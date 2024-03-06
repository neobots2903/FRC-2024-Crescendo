package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


/**
 * A command that will drive the robot forward a set distance using the swerve drive.
 */
public class DriveForwardPark extends Command
{

  private final SwerveSubsystem swerveSubsystem;
  private final PIDController   controller;

  public DriveForwardPark(SwerveSubsystem swerveSubsystem)
  {
    this.swerveSubsystem = swerveSubsystem;
    controller = new PIDController(1.0, 0.0, 0.0);
    controller.setSetpoint(0.0);
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    // Putting telemetry on Shuffleboard.
    SmartDashboard.putNumber("Auto X Position", swerveSubsystem.getPose().getX());
    SmartDashboard.putNumber("Auto Setpoint", controller.getSetpoint());

    // This line is getting the current position of the robot and setting the setpoint to 5 (units???)s
    double translationVal = controller.calculate(swerveSubsystem.getPose().getX(), 5.0);

    // This line tells the robot to move forward in the x direction.
    swerveSubsystem.drive(new Translation2d(translationVal, 0.0), 0.0, true);
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    // Once at the setpoint, the command will end.
    return controller.atSetpoint();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    // When the command ends, the robot will X-Stop.
    swerveSubsystem.lock();
  }
}