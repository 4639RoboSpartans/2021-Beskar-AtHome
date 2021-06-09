/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;
import frc.robot.subsystems.HopperSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.KickerSys;
import frc.robot.subsystems.ShooterSys;

import command.CommandBase;

public class PushBallsCmd extends CommandBase {
	private final HopperSys hopper;
	private final IntakeSys intake;
	private final ShooterSys shooter;

	public PushBallsCmd(HopperSys hopper, IntakeSys intake, ShooterSys shooter) {
		this.hopper = hopper;
		this.intake = intake;
		this.shooter = shooter;
		addRequirements(hopper, intake);
	}
	@Override
	public void initialize() {
			hopper.setHopper(0);
			intake.setIntake(0);
	}
	@Override
	public void execute() {
		if (shooter.isAtSpeed()) {
			hopper.setHopper(0.7);
			intake.setIntake(0.4);
		}
	}

	@Override
	public void end(boolean interrupted) {
		hopper.setHopper(0);
		intake.setIntake(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
