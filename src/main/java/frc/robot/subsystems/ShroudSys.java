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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import command.SubsystemBase;

public class ShroudSys extends SubsystemBase {
	private final WPI_VictorSPX shroud;
	final PIDController pid;
	public final Encoder shroudEncoder;
	public double positionDesired=0;
	public double pitch = 0;
	private double pidOut;
	public ShroudSys() {
		this.shroud = new WPI_VictorSPX(Constants.SHROUD_CAN);
		shroud.configFactoryDefault();
		shroud.setNeutralMode(NeutralMode.Brake);
		shroud.setInverted(InvertType.InvertMotorOutput);

		//Encoder initialization
		shroudEncoder = new Encoder(4,5, true);
		//shroudEncoder.reset();
		// PID Initialization
		this.pid = new PIDController(Constants.SHROUD_KP, Constants.SHROUD_KI, Constants.SHROUD_KD);
		this.pid.setSetpoint(0);
		pid.setTolerance(1);
	}

	public double getDegrees() {
		return shroudEncoder.getDistance();
	}
	public void setPos(double posDesired){
		positionDesired = posDesired;
	}
	/*public void setDesiredPosition(double pos) {
		if (pos == 0)
			positionDesired = Constants.SHROUD_PRESET_0;
		else if (pos == 1)
			positionDesired = Constants.SHROUD_PRESET_1;
		else if (pos == 2)
			positionDesired = Constants.SHROUD_PRESET_2;
		else if (pos == 3)
			positionDesired = Constants.SHROUD_PRESET_3;

		SmartDashboard.putString("DB/String 0", "DesPos: " + positionDesired);
	}*/

	public void setShroud(double power) {
		shroud.set(power);
	}

	@Override
	public void periodic() {
		pidOut = pid.calculate(pitch, 0) / 100.0;
		SmartDashboard.putNumber("Shroud Degrees:", getDegrees());
		pid.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0),
				SmartDashboard.getNumber("D", 0));
		if(pidOut<0&&getDegrees()>10){
			shroud.set(pidOut);
		}else if(pidOut>0&&getDegrees()<490){
			shroud.set(pidOut);
		}else{
			shroud.set(0);
		}
		 //uncomment for future use FOR MANUAL CONTROL OF SHROUD
		
	}
	public void resetEncoder()
	{
		shroudEncoder.reset();
	}
	public void setVolts(double volts){
		shroud.setVoltage(volts);
	}
}
