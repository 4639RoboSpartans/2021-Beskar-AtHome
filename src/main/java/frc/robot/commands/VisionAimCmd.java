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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.ShroudSys;
import frc.robot.subsystems.TurretSys;
import frc.robot.OI;
import frc.robot.Constants.Buttons;
import command.CommandBase;
//BUTTON TO USE THIS PROGRAM IS THE LEFT BUMPER ON THE SECOND REMOTE, OR REMOTE #1
public class VisionAimCmd extends CommandBase {
	private final TurretSys turret;
	private final ShroudSys shroud;
	public VisionAimCmd(TurretSys turret,ShroudSys shroud) {
		this.turret = turret;
		this.shroud = shroud;
		addRequirements(turret);
		if (Constants.STCam!=null){
			SmartDashboard.putBoolean("camera Found ", true);
		}else{
			SmartDashboard.putBoolean("camera Found", false);
		}
	}

	@Override
	//go to 10.46.39.11:5800 to see camera output and tune it
	/*
	things to complete:
	fine tune to inner goal
	fix shroud
	work on speed
	*/
	public void execute() {
		var result = Constants.STCam.getLatestResult();
		double KpRotTurret = 0.0035;
		double constantForceTurret = 0.0035;
		double KpRotShroud = -0.0055;//need to adjust
		double constantForceShroud = 0.0055;//need to adjust
		double angleTolerance = 0.25;// Deadzone for the angle control loop
		SmartDashboard.putBoolean("Target Aquired:", Constants.STCam.hasTargets());
		if(Constants.STCam.hasTargets()){
			double yaw = result.getBestTarget().getYaw();
			double pitch = result.getBestTarget().getPitch();
			if(Math.abs(yaw)>angleTolerance){
				turret.setTurret(KpRotTurret*yaw+constantForceTurret);
				SmartDashboard.putBoolean("Spinning", true);
			}else{
				turret.setTurret(0);
				SmartDashboard.putBoolean("Spinning",false);
			}
			/*SmartDashboard.putNumber("Shroud Degs", shroud.getDegrees());
			SmartDashboard.putNumber("pitch", pitch);
			if(shroud.getDegrees()>-5&&shroud.getDegrees()<500&&Math.abs(pitch)>angleTolerance){
				shroud.setShroud(KpRotShroud*pitch+constantForceShroud);
				SmartDashboard.putBoolean("Up down", true);
			}else{
				shroud.setShroud(0);
				SmartDashboard.putBoolean("Up down", false);
			}**/
		}
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
