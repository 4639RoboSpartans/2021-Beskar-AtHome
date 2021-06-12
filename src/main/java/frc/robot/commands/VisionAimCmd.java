/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSys;
import frc.robot.subsystems.ShroudSys;
import frc.robot.subsystems.TurretSys;

import command.CommandBase;
//BUTTON TO USE THIS PROGRAM IS THE LEFT BUMPER ON THE SECOND REMOTE, OR REMOTE #1
public class VisionAimCmd extends CommandBase {
	private final TurretSys turret;
	private final ShroudSys shroud; 
	private final PhotonVisionSys photon;
	public VisionAimCmd(TurretSys turret,ShroudSys shroud, PhotonVisionSys photon) {
		this.turret = turret;
		this.shroud = shroud;
		this.photon = photon;
		addRequirements(turret);
	}
	@Override
	//go to 10.46.39.11:5800 to see camera output and tune it
	public void execute() {
		if(photon.hasTargets()){
			double yaw = photon.getYaw();
			double pitch = photon.getPitch();
			if(Math.abs(yaw)>0){
				SmartDashboard.putBoolean("Spinning", true);
				if(yaw<0){
				turret.setTurret(Constants.KP_ROT_TURRET*yaw+Constants.CONSTANT_FORCE_TURRET);
				}else
				turret.setTurret(Constants.KP_ROT_TURRET*yaw-Constants.CONSTANT_FORCE_TURRET);
			}else{
				turret.resetTurret();
				SmartDashboard.putBoolean("Spinning",false);
			}
			shroud.pitch = pitch;
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
