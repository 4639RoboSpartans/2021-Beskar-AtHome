/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.DrivetrainSys;

import command.CommandBase;

public class ManualDriveCmd extends CommandBase {
	private final DrivetrainSys drive;
	private final OI oi;

	private final SlewRateLimiter speedLimiter = new SlewRateLimiter(10);//5
	private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(10);

	private final Encoder r_encoder;
	private final Encoder l_encoder;
	

	public ManualDriveCmd(DrivetrainSys drive, OI oi) {
		this.drive = drive;
		addRequirements(drive);
		this.oi = oi;
		this.r_encoder = drive.m_RightEncoder;
		this.l_encoder = drive.m_LeftEncoder;
	}

	@Override
	public void execute() {
		double reduceSpeed;

		if (oi.getAxis(0, Constants.Axes.RIGHT_TRIGGER) > 0)
			reduceSpeed = 0.7;
		else
			reduceSpeed = 1.0;

		SmartDashboard.putBoolean("Is In Drive Straight", oi.getAxis(0, Constants.Axes.RIGHT_STICK_X) == 0);
		SmartDashboard.putNumber("Error", 0);

		//If the right joystick is inside the deadzone run the direction correction to drive straight
		/*if(oi.getAxis(0, Constants.Axes.RIGHT_STICK_X) == 0)
		{
    		double error = l_encoder.getRate() - r_encoder.getRate();
			double turn_power = Constants.DRIVE_STRAIGHT_kP * error;
			drive.arcadeDrive(speedLimiter.calculate(oi.getAxis(0, Constants.Axes.LEFT_STICK_Y)) * reduceSpeed,
				rotationLimiter.calculate(turn_power));

				SmartDashboard.putNumber("Error", error);
				SmartDashboard.putNumber("Turn Power", turn_power);
		}
		else{*/
		drive.arcadeDrive(speedLimiter.calculate(oi.getAxis(0, Constants.Axes.LEFT_STICK_Y)) * reduceSpeed,
				rotationLimiter.calculate(oi.getAxis(0, Constants.Axes.RIGHT_STICK_X))
						* Constants.ROTATION_SENSITIVITY);	
		//}
		
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
