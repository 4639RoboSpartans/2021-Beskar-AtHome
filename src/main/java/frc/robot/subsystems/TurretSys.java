/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import command.SubsystemBase;

public class TurretSys extends SubsystemBase {
	private final WPI_TalonSRX turret;

	public TurretSys() {
		this.turret = new WPI_TalonSRX(Constants.TURRET_CAN);
		turret.configFactoryDefault();
		turret.setNeutralMode(NeutralMode.Brake);
		turret.setInverted(InvertType.InvertMotorOutput);
	}

	public void setTurret(double power) {
		turret.set(power);
	}

	public double getDegrees() {
		return turret.getSelectedSensorPosition();
	}
	public void resetTurret(){
		double defAngle = 2000;
		double degs = getDegrees()+defAngle;
		if(degs<0){
			setTurret(Constants.KP_ROT_TURRET*(degs/25)+Constants.CONSTANT_FORCE_TURRET);
		}else
			setTurret(Constants.KP_ROT_TURRET*(degs/25)-Constants.CONSTANT_FORCE_TURRET);
	}
	public void setTurretPos(double pos){
		int posOffset = 0;
		double degs = getDegrees()+posOffset;
		if(degs<pos){
			setTurret(Constants.KP_ROT_TURRET*(degs/25)+Constants.CONSTANT_FORCE_TURRET);
		}else
			setTurret(Constants.KP_ROT_TURRET*(degs/25)-Constants.CONSTANT_FORCE_TURRET);
	}
	@Override
	public void periodic() {
	}
}
