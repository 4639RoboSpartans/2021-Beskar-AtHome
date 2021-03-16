package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import command.SubsystemBase;

public class IntakeElevator extends SubsystemBase {
	private final WPI_TalonSRX intakePivot;

	public IntakeElevator() {

		this.intakePivot = new WPI_TalonSRX(Constants.INTAKE_PIVOT_CAN);
		intakePivot.configFactoryDefault();
		intakePivot.configContinuousCurrentLimit(10);
		intakePivot.setNeutralMode(NeutralMode.Brake);
	}
	/*
	 * private double getDegrees() { return intakePivot.getSelectedSensorPosition()
	 * / 4096.0 + 90; }
	 */

	
	public void setPivot(double num) {
		intakePivot.set(num);
	}

	@Override
	public void periodic() {
	}
}