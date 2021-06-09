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

import command.SubsystemBase;

public class KickerSys extends SubsystemBase {
	private final WPI_TalonSRX kicker;
	private double currentSpeed;
	public KickerSys() {
		this.kicker = new WPI_TalonSRX(Constants.KICKER_CAN);
		kicker.configFactoryDefault();
		kicker.setNeutralMode(NeutralMode.Coast);
		kicker.setSensorPhase(true);
		currentSpeed =0;
	}

	public void setKicker(double num) {
		currentSpeed = num;
	}
	public void setKickerVolts(double volts){
		kicker.setVoltage(volts);
	}
	public boolean uptoSpeed(){
		return currentSpeed>0;
	}
	@Override
	public void periodic() {
	}
}
