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

import org.photonvision.PhotonUtils;

import command.CommandBase;
//BUTTON TO USE THIS PROGRAM IS THE LEFT BUMPER ON THE SECOND REMOTE, OR REMOTE #1
public class VisionAimCmd extends CommandBase {
	private final TurretSys turret;
	private final ShroudSys shroud;
	private static final double CameraHeight = 0;//assign values
	private static  double CameraPitch = 0;//assign values
	private static final double TargetHeight = 2.49555;
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
	distance is 5.5 in
	set conditition to not allow intake until flywheel up to speed
	*/
	public void execute() {
		var result = Constants.STCam.getLatestResult();
		double KpRotTurret = 0.0035;
		double constantForceTurret = 0.0035;
		double KpRotShroud = -0.0055;//need to adjust
		double constantForceShroud = 0.0055;//need to adjust
		double angleTolerance = 0;// Deadzone for the angle control loop
		SmartDashboard.putBoolean("Target Aquired:", Constants.STCam.hasTargets());
		if(Constants.STCam.hasTargets()){
			double yaw = result.getBestTarget().getYaw();
			double pitch = result.getBestTarget().getPitch();
			double distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(CameraHeight, TargetHeight, 
										Math.toRadians(CameraPitch+((shroud.shroudEncoder.getRaw()+0.0)/256)*360), Math.toRadians(pitch));
			if(Math.abs(yaw)>angleTolerance){
				//turret.setTurret(KpRotTurret*yaw+constantForceTurret);
				//check to see if there is a encoder and reset pos after turning
				SmartDashboard.putBoolean("Spinning", true);
				if(yaw<0){
					turret.setTurret(KpRotTurret*yaw-constantForceTurret);
				}else{
					turret.setTurret(KpRotTurret*yaw+constantForceTurret);
				}
			}else{
				turret.setTurret(0);
				SmartDashboard.putBoolean("Spinning",false);
			}

			//commenting RC 108-127 138 139 197-207 Shroudsys 52-63 71-78 39-45
			if(shroud.getDegrees()>-5&&shroud.getDegrees()<500&&Math.abs(pitch)>angleTolerance){
				shroud.setShroud(KpRotShroud*pitch+constantForceShroud);
				SmartDashboard.putBoolean("Up down", true);
				if(pitch<0){
					shroud.setShroud(KpRotShroud*pitch-constantForceShroud);
				}else{
					shroud.setShroud(KpRotShroud*pitch+constantForceShroud);
				}
			}else{
				//reset the shroud to original pos
				/*if(shroud.getDegrees()>0){
					shroud.setShroud(KpRotShroud*shroud.getDegrees()+constantForceShroud);
				}*/
				shroud.setShroud(0);
				SmartDashboard.putBoolean("Up down", false);
			}
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
