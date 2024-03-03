// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem.IntakeDirection;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystemSimple;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  private final IntakeShooterSubsystem m_intakeShooter = new IntakeShooterSubsystem();

  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ArmSubsystemSimple m_armSimple = new ArmSubsystemSimple();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  final CommandXboxController operatorXbox = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX());

    m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // ---------- DRIVER bindings -----------------------
    driverXbox.a().onTrue((Commands.runOnce(m_drivebase::zeroGyro)));
    driverXbox.x().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());

    // Drive slower when left bumper is pressed, faster when right bumper is pressed.
    driverXbox
        .leftBumper()
        .onTrue(Commands.runOnce(() -> m_drivebase.maximumSpeed = 0.25))
        .onFalse(Commands.runOnce(() -> m_drivebase.maximumSpeed = 0.625));

    driverXbox
        .rightBumper()
        .onTrue(Commands.runOnce(() -> m_drivebase.maximumSpeed = 0.75))
        .onFalse(Commands.runOnce(() -> m_drivebase.maximumSpeed = 0.625));

    // --------------------------------------------------

    // --------- OPERATOR bindings ----------------------
    operatorXbox.y().onTrue(new InstantCommand(() -> m_intakeShooter.startShooter()))
        .onFalse(new InstantCommand(() -> m_intakeShooter.stopShooter()));

    operatorXbox.b().onTrue(new InstantCommand(() -> m_intakeShooter.startIntake(IntakeDirection.IN, IntakeShooterConstants.kIntakeSpeed)))
        .onFalse(new InstantCommand(() -> m_intakeShooter.stopIntake()));

      operatorXbox.x().onTrue(new InstantCommand(() -> m_intakeShooter.startIntake(IntakeDirection.OUT, IntakeShooterConstants.kIntakeSpeed)))
        .onFalse(new InstantCommand(() -> m_intakeShooter.stopIntake()));

    // Arm extension, right up, left down.
    // operatorXbox.rightBumper().onTrue(new InstantCommand(() -> m_arm.extendArm()));
    // operatorXbox.leftBumper().onTrue(new InstantCommand(() -> m_arm.retractArm()));

    // Disable the arm controller when Left is pressed.
    operatorXbox.povLeft().onTrue(Commands.runOnce(m_arm::disable));

    // Arm Intake position when Down is pressed.
    operatorXbox.povDown().onTrue(
      Commands.runOnce(
          () -> {
            // If the arm is extended, set the goal to the intake position, don't kill swerve.
            if (m_arm.isExtended()) {
              m_arm.setGoal(Constants.ArmConstants.kArmIntakePosition);
              m_arm.enable();
            } else {
              m_arm.setGoal(Constants.ArmConstants.kArmRestingPosition);
              m_arm.enable();
            }
          },
          m_arm));

    // Arm Speaker position when Right is pressed.
    operatorXbox.povRight().onTrue(
      Commands.runOnce(
          () -> {
            m_arm.setGoal(Constants.ArmConstants.kArmAmpPosition);
            m_arm.enable();
          },
          m_arm));

    // Arm Amp position when Up is pressed.
    operatorXbox.povUp().onTrue(
      Commands.runOnce(
          () -> {
            m_arm.setGoal(Constants.ArmConstants.kArmSpeakerPosition);
            m_arm.enable();
          },
          m_arm));

    // ADDED CODE FOR JUST P LOOP TESTING
    operatorXbox.rightBumper().onTrue(
      Commands.runOnce(
          () -> {
            m_armSimple.goToPosition(ArmConstants.kArmSpeakerPosition);
          },
          m_arm));

    operatorXbox.leftBumper().onTrue(
      Commands.runOnce(
          () -> {
            m_armSimple.disableArm();
          },
          m_arm));
    // --------------------------------------------------
  }

  public void setMotorBrake(boolean brake)
  {
    m_drivebase.setMotorBrake(brake);
  }
}