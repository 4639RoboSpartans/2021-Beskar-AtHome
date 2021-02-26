/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants;
import frc.robot.subsystems.TurretSys;

import command.CommandBase;

public class VisionAimCmd extends CommandBase {
	private final TurretSys turret;
	private final NetworkTableEntry yawEntry;

	public VisionAimCmd(TurretSys turret) {
		this.turret = turret;
		addRequirements(turret);
		NetworkTable chameleonTable = NetworkTableInstance.getDefault().getTable("chameleon-vision")
				.getSubTable(Constants.TURRET_CAMERA_NAME);
		this.yawEntry = chameleonTable.getEntry("yaw");
	}

	@Override
	public void execute() {
		double KpRot = -0.1;
		double constantForce = 0.05;
		double angleTolerance = 5;// Deadzone for the angle control loop

		double rotationError = yawEntry.getDouble(0.0);
		if (rotationError > angleTolerance)
			turret.setTurret(KpRot * rotationError + constantForce);
		else
			turret.setTurret(0);
	}

	@Override
	public void end(boolean interrupted) {
		turret.setTurret(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
