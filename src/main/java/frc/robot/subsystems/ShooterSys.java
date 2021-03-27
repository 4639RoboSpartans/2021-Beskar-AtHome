/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import command.SubsystemBase;

public class ShooterSys extends SubsystemBase {
	private final WPI_TalonSRX topShooter;
	private final WPI_VictorSPX bottomShooter;

	private final PIDController pid;

	private double speedDesired = 0;

	public ShooterSys() {
		this.topShooter = new WPI_TalonSRX(Constants.TOP_SHOOTER_CAN);
		topShooter.configFactoryDefault();
		topShooter.setNeutralMode(NeutralMode.Brake);
		topShooter.setSensorPhase(true);

		this.bottomShooter = new WPI_VictorSPX(Constants.BOTTOM_SHOOTER_CAN);
		bottomShooter.configFactoryDefault();
		bottomShooter.setNeutralMode(NeutralMode.Brake);
		bottomShooter.follow(topShooter);

		this.pid = new PIDController(Constants.SHOOTER_KP, 0, 0);
		pid.setTolerance(3);
	}

	private double getSpeed() {
		// rotations per second
		return topShooter.getSelectedSensorVelocity() / (4096.0 * 16.0 / 36.0) * 10;
	}

	public boolean isAtSpeed() {
		return speedDesired != 0 && pid.atSetpoint();
	}

	public void setShooter(double speed) {
		// rotations per second
		speedDesired = speed;
	}

	@Override
	public void periodic() {
		double pidOut = pid.calculate(getSpeed(), speedDesired) + Constants.SHOOTER_FEEDFORWARD.calculate(speedDesired);
		SmartDashboard.putNumber("Shooter PID", pidOut);
		SmartDashboard.putNumber("Shooter Speed", topShooter.getSelectedSensorVelocity());
		SmartDashboard.putNumber("Shooter DesiredSpeed", this.speedDesired);
		if (speedDesired == 0) {
			pid.reset();
			topShooter.set(0);
		} else {
			topShooter.setVoltage(pidOut);
		}
	}
}
