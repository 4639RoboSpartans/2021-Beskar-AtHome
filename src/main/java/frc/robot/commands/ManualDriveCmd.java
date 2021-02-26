/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.DrivetrainSys;

import command.CommandBase;

public class ManualDriveCmd extends CommandBase {
	private final DrivetrainSys drive;
	private final OI oi;

	private final SlewRateLimiter speedLimiter = new SlewRateLimiter(5);
	private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(5);

	public ManualDriveCmd(DrivetrainSys drive, OI oi) {
		this.drive = drive;
		addRequirements(drive);
		this.oi = oi;
	}

	@Override
	public void execute() {
		double reduceSpeed;

		if (oi.getAxis(0, Constants.Axes.RIGHT_TRIGGER) > 0)
			reduceSpeed = 0.65;
		else
			reduceSpeed = 1.0;

		drive.arcadeDrive(speedLimiter.calculate(oi.getAxis(0, Constants.Axes.LEFT_STICK_Y)) * reduceSpeed,
				rotationLimiter.calculate(oi.getAxis(0, Constants.Axes.RIGHT_STICK_X))
						* Constants.ROTATION_SENSITIVITY);
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
