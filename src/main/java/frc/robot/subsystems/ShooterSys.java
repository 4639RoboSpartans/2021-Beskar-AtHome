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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
		topShooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100); //Timeout 100ms

		this.bottomShooter = new WPI_VictorSPX(Constants.BOTTOM_SHOOTER_CAN);
		bottomShooter.configFactoryDefault();
		bottomShooter.setNeutralMode(NeutralMode.Brake);
		bottomShooter.follow(topShooter);

		this.pid = new PIDController(Constants.SHOOTER_KP, 0, 0);
		pid.setTolerance(3);
	}

	public double getSpeed() {
		// rotations per second
		return topShooter.getSelectedSensorVelocity() * (600.0 / 4096) ; //(4096.0 * 16.0 / 36.0) * 10;
	}

	public boolean isAtSpeed() {
		return getSpeed() >2200;
	}

	public void setShooter(double speed) {
		// rotations per second
		speedDesired = speed;
	}

	@Override
	public void periodic() {
		if (speedDesired == 0) {
			pid.reset();
			topShooter.set(0);
		} else {
			double currentVoltage = speedDesired/2800*12;//pid.calculate(getSpeed(), speedDesired) + Constants.SHOOTER_FEEDFORWARD.calculate(speedDesired);
			SmartDashboard.putNumber("VOLTAGE TO SHOOTER", currentVoltage);
			topShooter.setVoltage(currentVoltage);
		}
		SmartDashboard.putNumber("Shooter_SpeedDesired", speedDesired);
		SmartDashboard.putNumber("Shooter_SpeedActual", getSpeed());


	}
}
