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
import frc.robot.subsystems.TurretSys;
import frc.robot.OI;
import frc.robot.Constants.Buttons;
import command.CommandBase;

public class VisionAimCmd extends CommandBase {
	private final TurretSys turret;
	public VisionAimCmd(TurretSys turret) {
		this.turret = turret;
		addRequirements(turret);
		if (Constants.STCam!=null){
			SmartDashboard.putBoolean("camera Found ", true);
		}else{
			SmartDashboard.putBoolean("camera Found", false);
		}
	}

	@Override
	//go to photonvisionslhs.local:5800 to see camera output and tune it
	public void execute() {
		var result = Constants.STCam.getLatestResult();
		double KpRot = 0.005;
		double constantForce = 0.005;
		double angleTolerance = 5;// Deadzone for the angle control loop
		SmartDashboard.putBoolean("Target Aquired:", Constants.STCam.hasTargets());
		if(Constants.STCam.hasTargets()){
			double yaw = result.getBestTarget().getYaw();
			if(Math.abs(yaw)>angleTolerance){
				turret.setTurret(KpRot*yaw+constantForce);
				SmartDashboard.putBoolean("Spinning", true);
			}else{
			
				turret.setTurret(0);
				SmartDashboard.putBoolean("Spinning",false);
			}
		}
		/*if (rotationError > angleTolerance)
			turret.setTurret(KpRot * rotationError + constantForce);
		else
			turret.setTurret(0);*/
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
