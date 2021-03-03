/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import command.Command;
import command.CommandScheduler;
import command.WaitCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();

		CameraServer.getInstance().startAutomaticCapture();
		SmartDashboard.putNumber("time1", m_robotContainer.time1);
		SmartDashboard.putNumber("time2", m_robotContainer.time2);
		SmartDashboard.putNumber("time3", m_robotContainer.time3);
		SmartDashboard.putNumber("time4", m_robotContainer.time4);
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		m_robotContainer.setDriveNeutralMode(NeutralMode.Brake);
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		/*//run the intake
		m_robotContainer.m_intake.setIntake(0.5);
		//go forward
		m_robotContainer.m_drive.arcadeDrive(0.5, 0);
		new WaitCommand(0.5);
		//stop and turn
		m_robotContainer.m_drive.arcadeDrive(0, 0);
		m_robotContainer.m_drive.arcadeDrive(0.5, 45);
		//go forward
		m_robotContainer.m_drive.arcadeDrive(0.5, 0);
		new WaitCommand(0.5);
		//stop and turn
		m_robotContainer.m_drive.arcadeDrive(0, 0);
		m_robotContainer.m_drive.arcadeDrive(0.5, -90);
		//go forward
		m_robotContainer.m_drive.arcadeDrive(0.5, 0);
		new WaitCommand(0.5);
		//stop and turn
		m_robotContainer.m_drive.arcadeDrive(0, 0);
		m_robotContainer.m_drive.arcadeDrive(0.5, 45);
		//go forards
		m_robotContainer.m_drive.arcadeDrive(0.5, 0);
		new WaitCommand(0.5);*/

	}

	@Override
	public void teleopInit() {
		m_robotContainer.setDriveNeutralMode(NeutralMode.Brake);
		m_robotContainer.m_drive.m_leftEncoder.reset();
		m_robotContainer.m_drive.m_rightEncoder.reset();
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putNumber("LEncoder ", m_robotContainer.m_drive.m_leftEncoder.getDistance());
		SmartDashboard.putNumber("REncoder ", m_robotContainer.m_drive.m_rightEncoder.getDistance());
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
