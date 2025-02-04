// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.qrfc.Reef;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	private final SendableChooser<Command> autoChooser;
	private Reef reef = new Reef();


	// Replace with CommandPS4Controller or CommandJoystick if needed
	final CommandPS5Controller driverController = new CommandPS5Controller(0);
	// The robot's subsystems and commands are defined here...
	private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
			"swerve/Software-bot-swerve-config"));

	Pose2d targetPose = drivebase.getPose();

	/**
	 * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
	 */
	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
			drivebase.getSwerveDrive(),
			() -> driverController.getLeftY() * -1,
			() -> driverController.getLeftX() * -1
	).withControllerRotationAxis(
			() -> driverController.getRightX() * -1)
			.deadband(OperatorConstants.DEADBAND)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
	 */
	SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
			driverController::getRightX,
			driverController::getRightY
	).headingWhile(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
	 */
	SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
			.allianceRelativeControl(false);

	SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> -driverController.getLeftY(),
			() -> -driverController.getLeftX()
	).withControllerRotationAxis(()
					-> driverController.getRawAxis(2))
			.deadband(OperatorConstants.DEADBAND)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);

	// Derive the heading axis with math!
	SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy().withControllerHeadingAxis(
			() -> Math.sin(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
			() -> Math.cos(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2)
	).headingWhile(true);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		DriverStation.silenceJoystickConnectionWarning(true);
		NamedCommands.registerCommand("test", Commands.print("I EXIST"));


		boolean isCompetition = true;

		// Build an auto chooser. This will use Commands.none() as the default option.
		// As an example, this will only show autos that start with "comp" while at
		// competition as defined by the programmer
		autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
				(stream) -> isCompetition
						? stream.filter(auto -> auto.getName().startsWith("comp"))
						: stream
		);
		autoChooser.addOption("2m", new PathPlannerAuto("2m"));
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
	 * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
	 * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
	 */
	private void configureBindings() {
		Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
		Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
		Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
		Command driveSetpointGen                   = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
		Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
		Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
		Command driveSetpointGenKeyboard                   = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

		if (RobotBase.isSimulation()) {
			drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
		} else {
			drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
		}

		if (Robot.isSimulation()) {
			driverController.options().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
			driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
		}
		if (DriverStation.isTest()) {
			drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

			driverController.square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
			driverController.triangle().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
			driverController.options().onTrue((Commands.runOnce(drivebase::zeroGyro)));
			driverController.create().whileTrue(drivebase.centerModulesCommand());
			driverController.L1().onTrue(Commands.none());
			driverController.R1().onTrue(Commands.none());
		} else {
			driverController.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
			driverController.square().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
			driverController.circle().onTrue(new InstantCommand(() -> {
				targetPose = drivebase.getPose();
				double[] poseArray = {
					targetPose.getX(),
					targetPose.getY(),
					targetPose.getRotation().getDegrees()
				};
				SmartDashboard.putNumberArray("Target Pose", poseArray);
				SmartDashboard.putNumber("Target pose error", targetPose.getTranslation().getDistance(drivebase.getPose().getTranslation()));
			}, drivebase));

			driverController.triangle().whileTrue(
				new DeferredCommand(
					() -> new ParallelCommandGroup(
						drivebase.driveToPose(targetPose),
						Commands.run(() ->
							SmartDashboard.putNumber("Target pose error", targetPose.getTranslation().getDistance(drivebase.getPose().getTranslation()))
						)
					), Set.of()
				)
			);

			driverController.options().whileTrue(Commands.none());
			driverController.create().whileTrue(Commands.none());
			driverController.L1().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
			driverController.R1().onTrue(Commands.none());
		}
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
//		return drivebase.getAutonomousCommand("L4-1");
		//return drivebase.getAutonomousCommand("2m");
		return autoChooser.getSelected();

	}

	public void setMotorBrake(boolean brake)
	{
		drivebase.setMotorBrake(brake);
	}
}