/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import command.SubsystemBase;

public class HopperSys extends SubsystemBase {
	private final WPI_VictorSPX hopper;

	public HopperSys() {
		this.hopper = new WPI_VictorSPX(Constants.HOPPER_CAN);
		hopper.configFactoryDefault();
		hopper.setNeutralMode(NeutralMode.Brake);
	}

	public void setHopper(double num) {
		hopper.set(num);
	}

	@Override
	public void periodic() {
	}
}
