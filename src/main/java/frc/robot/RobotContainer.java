// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();

	private final CANSparkMax floorSuck = new CANSparkMax(9, MotorType.kBrushless);

	private final CANSparkMax armMove0 = new CANSparkMax(10, MotorType.kBrushless);
	private final CANSparkMax armMove1 = new CANSparkMax(11, MotorType.kBrushless);
	private final CANSparkMax armSuck = new CANSparkMax(12, MotorType.kBrushless);
	private final CANSparkMax armShoot0 = new CANSparkMax(13, MotorType.kBrushless);
	private final CANSparkMax armShoot1 = new CANSparkMax(14, MotorType.kBrushless);

	// THE MOVE CONTROLLER MUST BE SET TO D-PAD
	// THE NOTE CONTROLLER MUST BE SET TO XBOX

	XboxController controller = new XboxController(0);

	public RobotContainer() {
		configureButtonBindings();

		m_robotDrive.setDefaultCommand(
			new RunCommand(
				() -> {
					m_robotDrive.drive(
						-MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband),
						-MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband),
						-MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband),
						true, true
					);

					//Double slowNote = 0.25;
					Double fastNote = 0.55;

					floorSuck.set(controller.getYButton() ? 0.5 : (controller.getAButton() ? -0.5 : 0));

					armSuck.set(controller.getXButton() ? 0.4 : (controller.getYButton() ? -0.4 : 0));

					armShoot0.set(controller.getBButton() ? -fastNote : 0);
					armShoot1.set(controller.getBButton() ? -fastNote : 0);

					//System.out.println(armMove0.getEncoder().getPosition());

					armMove0.set(controller.getLeftTriggerAxis() > 0.5 ? 0.3 : (controller.getRightTriggerAxis() > 0.5 ? -0.3 : 0));
					armMove1.set(controller.getLeftTriggerAxis() > 0.5 ? -0.3 : (controller.getRightTriggerAxis() > 0.5 ? 0.3 : 0));
					
					// Y = front arm and back arm shoot/suck
					// B = back arm shoot/suck
					// X = top remove
					// A = bottom remove
				},
				m_robotDrive
			)
		);
	}

	private void configureButtonBindings() {
		new JoystickButton(controller, Button.kR1.value).whileTrue(new RunCommand(
			() -> m_robotDrive.setX(),
			m_robotDrive
		));
	}

	public double command(double[] speeds) {
		return command(speeds, "");
	}

	public double command(double[] speeds, String cmd) {
		double shootSpeed = 0.55;
		double ret = 0;

		switch (cmd) {
			case "fire0":
				speeds[2] = -shootSpeed;
				speeds[3] = -shootSpeed;
				break;
			
			case "fire1":
				speeds[1] = -shootSpeed;
				break;
			
			case "suck":
				speeds[0] = 0.5;
				speeds[1] = -0.4;
				break;

			case "stopArm":
				speeds[0] = 0.0;
				speeds[1] = 0.0;
				speeds[2] = 0.0;
				speeds[3] = 0.0;
				break;

			case "stopAll":
				speeds[0] = 0.0;
				speeds[1] = 0.0;
				speeds[2] = 0.0;
				speeds[3] = 0.0;
				speeds[4] = 0.0;
				speeds[5] = 0.0;
				speeds[6] = 0.0;
				break;
			
			case "forwards":
				speeds[4] = -0.2;
				speeds[6] = 0.0;
				break;

			case "backwards":
				speeds[4] = 0.2;
				speeds[6] = 0.0;
				break;

			case "straighten":
				speeds[6] = 1;
				break;

			case "shortWait":
				ret = 1;
				break;
			
			case "longWait":
				ret = 3;
				break;
			
			default:
				if (cmd != "") System.out.println("command " + cmd + " not found!");
				break;
		}

		
		floorSuck.set(speeds[0]);
		armSuck.set(speeds[1]);
		armShoot0.set(speeds[2]);
		armShoot1.set(speeds[3]);

		double s = 0;

		if (speeds[6] > 0.5) {
			if (m_robotDrive.getPose().getY() > 0.2) {
				s = -0.2;
			} else if (m_robotDrive.getPose().getY() < -0.2) {
				s = 0.2;
			}
		}

		m_robotDrive.drive(
			-MathUtil.applyDeadband(speeds[4], OIConstants.kDriveDeadband),
			-MathUtil.applyDeadband(speeds[5], OIConstants.kDriveDeadband),
			-MathUtil.applyDeadband(s, OIConstants.kDriveDeadband),
			false, true
		);

		return ret;
	}

	/*public Command getAutonomousCommand() {
		Double shootSpeed = 0.55;

		return new RunCommand(() -> {
			armShoot0.set(-shootSpeed);
			armShoot1.set(-shootSpeed);
		}).andThen(new WaitCommand(1).andThen(new RunCommand(() -> armSuck.set(-shootSpeed)).andThen(new WaitCommand(0.5).andThen(new RunCommand(() -> {
			armShoot0.set(0);
			armShoot1.set(0);
			armSuck.set(0);
		})))));
	}*/

	public boolean isRobotAtPosition(Double x, Double y) {
		Pose2d pose = m_robotDrive.getPose();

		return Math.abs(pose.getX() - x) < 0.2 && Math.abs(pose.getY() - y) < 0.2;
	}
}
