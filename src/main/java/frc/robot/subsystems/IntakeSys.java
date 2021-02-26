/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import command.SubsystemBase;

public class IntakeSys extends SubsystemBase {
	private final WPI_VictorSPX intakeWheel;
	private final WPI_TalonSRX intakePivot;

	public IntakeSys() {
		this.intakeWheel = new WPI_VictorSPX(Constants.INTAKE_WHEEL_CAN);
		intakeWheel.configFactoryDefault();
		intakeWheel.setNeutralMode(NeutralMode.Brake);

		this.intakePivot = new WPI_TalonSRX(Constants.INTAKE_PIVOT_CAN);
		intakePivot.configFactoryDefault();
		intakePivot.configContinuousCurrentLimit(10);
		intakePivot.setNeutralMode(NeutralMode.Brake);
	}
	/*
	 * private double getDegrees() { return intakePivot.getSelectedSensorPosition()
	 * / 4096.0 + 90; }
	 */

	public void setIntake(double num) {
		intakeWheel.set(num);
	}

	public void setPivot(double num) {
		intakePivot.set(num);
	}

	@Override
	public void periodic() {
	}
}
