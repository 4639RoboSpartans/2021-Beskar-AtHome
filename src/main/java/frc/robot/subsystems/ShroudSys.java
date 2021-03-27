/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
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
	private final Encoder shroudEncoder;
	private double positionDesired;
	private double previouPosition;
	private int shroudReset = 10;
	private double pidOut;

	public ShroudSys() {
		this.shroud = new WPI_TalonSRX(Constants.SHROUD_CAN);
		shroud.configFactoryDefault();
		shroud.setNeutralMode(NeutralMode.Brake);
		shroud.setInverted(InvertType.InvertMotorOutput);

		//Encoder initialization
		shroudEncoder = new Encoder(4,5, true);
		shroudEncoder.reset();
		// PID Initialization
		this.pid = new PIDController(Constants.SHROUD_KP, Constants.SHROUD_KI, 0);
		this.pid.setSetpoint(0);
		pid.setTolerance(1);
		SmartDashboard.putNumber("P", Constants.SHROUD_KP);
		SmartDashboard.putNumber("I", Constants.SHROUD_KI);
		SmartDashboard.putNumber("D", 0);
	}

	public double getDegrees() {
		return shroudEncoder.getDistance();
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
		if (positionDesired != previouPosition && shroudReset > 0) {
			shroudReset--;
			pidOut = pid.calculate(getDegrees(), Constants.SHROUD_MAX_POS) / 1000.0;
		} else {
			pidOut = pid.calculate(getDegrees(), positionDesired) / 1000.0;
			shroudReset = 10;
			previouPosition = positionDesired;
		}
		pid.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0),
				SmartDashboard.getNumber("D", 0));
		SmartDashboard.putString("DB/String 6", "ShroudPeriodic " + getDegrees());
		SmartDashboard.putString("DB/String 7", "Setpoint: " + pid.atSetpoint());
		SmartDashboard.putString("DB/String 8", "PidOut: " + pidOut);
		SmartDashboard.putNumber("PIDOut", pidOut);
		SmartDashboard.putNumber("ShroudError", getDegrees() - positionDesired);
		shroud.set(pidOut);

	}

	public void resetEncoder()
	{
		shroudEncoder.reset();
	}
}
