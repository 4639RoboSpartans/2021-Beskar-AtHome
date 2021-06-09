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
import edu.wpi.first.wpilibj.DriverStation;
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
	private static final double CameraHeight = 0.6223;//assign values in meters
	private static  double CameraPitch = 0.8727;//assign values in radians
	private static final double TargetHeight = 2.49555;
	private static final double DistTurMidToCam = 0.1397;//5.5 inches in meters
	private double angleTolerance = 0;// Deadzone for the angle control loop
	public double pitchOff = -30;
	public double yawOff = -10;
	public VisionAimCmd(TurretSys turret,ShroudSys shroud) {
		this.turret = turret;
		this.shroud = shroud;
		//var result = null;
		addRequirements(turret);
		if (Constants.STCam!=null){
			SmartDashboard.putBoolean("camera Found ", true);
			//result = Constants.STCam.getLatestResult();
		}else{
			SmartDashboard.putBoolean("camera Found", false);
		}
		
		//double pitch = result.getBestTarget().getPitch();
		
	}
	public void setPitch(double pitch){
		pitchOff = pitch;
	}
	public void setYaw(double yaw){
		yawOff = yaw;
	}
	@Override
	//go to 10.46.39.11:5800 to see camera output and tune it
		/*ToDo:
		bring extra cam for climber
		finetune aiming and shooting(mostly, if not already, done) 
		auto cam stream switch
		*/

	public void execute() {
		var result = Constants.STCam.getLatestResult();
		//double KpRotShroud = -0.007;//need to adjust
		//double constantForceShroud = 0.007;//need to adjust
		SmartDashboard.putBoolean("Target Aquired:", Constants.STCam.hasTargets());
		setPitch(SmartDashboard.getNumber("PitchOffSet",pitchOff));
		setYaw(SmartDashboard.getNumber("YawoffSet",yawOff));
		if(Constants.STCam.hasTargets()){
			double yaw = result.getBestTarget().getYaw();
			double pitch = result.getBestTarget().getPitch();
			double area = result.getBestTarget().getArea();
			double distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(CameraHeight, TargetHeight, 
										Math.toRadians(CameraPitch+((shroud.getDegrees()+0.0)/500)*360), Math.toRadians(pitch));
			SmartDashboard.putNumber("distancetoTarget", distanceToTarget);
			double additionalAngle = Math.toDegrees((Math.atan(Math.toRadians(DistTurMidToCam)/Math.toRadians(distanceToTarget))));//contains the angle offset
			SmartDashboard.putNumber("ANGlEoffset", additionalAngle);
			//if(distanceToTarget)
			yaw+=yawOff;
			pitch+=pitchOff;	 
			if(Math.abs(yaw)>angleTolerance){
				//turret.setTurret(KpRotTurret*yaw+constantForceTurret);
				//check to see if there is a encoder and reset pos after turning
				SmartDashboard.putBoolean("Spinning", true);
				//double DesiredPos = yaw/0.008;
				//turret.setTurretPos(DesiredPos);
				if(yaw<0){
				turret.setTurret(Constants.KP_ROT_TURRET*yaw+Constants.CONSTANT_FORCE_TURRET);
				}else
				turret.setTurret(Constants.KP_ROT_TURRET*yaw-Constants.CONSTANT_FORCE_TURRET);
			}else{
				turret.resetTurret();
				SmartDashboard.putBoolean("Spinning",false);
			}

			//commenting RC 108-127 138 139 197-207 Shroudsys 52-63 71-78 39-45
			shroud.pitch = pitch;
			/*if(shroud.getDegrees()>-5&&shroud.getDegrees()<500&&Math.abs(pitch)>angleTolerance){
				//double temp = shroud.getDegrees()+((pitch*-1)/360)*500;
				//shroud.setShroud(KpRotShroud*pitch+constantForceShroud);
				//shroud.positionDesired = temp;
				
			}*//*else if(shroud.getDegrees()<0){
				shroud.resetEncoder();
			}*/
			/*else{
				//shroud.setShroud(0);
			}*/
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
