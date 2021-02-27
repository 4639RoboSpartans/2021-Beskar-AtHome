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

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import command.SubsystemBase;

public class ShroudSys extends SubsystemBase {
	private final WPI_TalonSRX shroud;
	private final PIDController pid;
	private double positionDesired;
	private double pidOut;

	public ShroudSys() {
		this.shroud = new WPI_TalonSRX(Constants.SHROUD_CAN);
		shroud.configFactoryDefault();
		shroud.setNeutralMode(NeutralMode.Brake);
		shroud.setInverted(InvertType.InvertMotorOutput);

		// PID Initialization
		this.pid = new PIDController(Constants.SHROUD_KP, Constants.SHROUD_KI, 0);
		this.pid.setSetpoint(0);
		pid.setTolerance(10);
		SmartDashboard.putNumber("P", Constants.SHROUD_KP);
		SmartDashboard.putNumber("I", Constants.SHROUD_KI);
		SmartDashboard.putNumber("D", 0);
		shroud.setSelectedSensorPosition(0);
	}

	public double getDegrees() {
		return shroud.getSelectedSensorPosition();
	}

	public void setDesiredPosition(double pos) {
		if (pos == 0)
			positionDesired = Constants.SHROUD_PRESET_0;
		else if (pos == 1)
			positionDesired = Constants.SHROUD_PRESET_1;
		else if (pos == 2)
			positionDesired = Constants.SHROUD_PRESET_2;
		else if (pos == 3)
			positionDesired = Constants.SHROUD_PRESET_3;

		SmartDashboard.putString("DB/String 0", "DesPos: " + positionDesired);
	}

	public void setShroud(double power) {
		shroud.set(power);
	}

	@Override
	public void periodic() {
		pidOut = pid.calculate(getDegrees(), positionDesired) / 1000.0;

		pid.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0),
				SmartDashboard.getNumber("D", 0));
		SmartDashboard.putString("DB/String 6", "ShroudPeriodic " + getDegrees());
		SmartDashboard.putString("DB/String 7", "Setpoint: " + pid.atSetpoint());
		//SmartDashboard.putString("DB/String 8", "PidOut: " + pidOut);
		shroud.set(pidOut);
		// shroud.set(MathUtil.clamp(pidOut, -1.0, 1.0));

	}
}
