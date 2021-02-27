/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import command.SubsystemBase;

public class DrivetrainSys extends SubsystemBase {
	private final WPI_VictorSPX frontLeft, frontRight, backLeft, backRight;
	private final AHRS navx;
	private final DifferentialDriveOdometry odometry;
	private final DifferentialDrive drive;
	private final Encoder m_leftEncoder;
	private final Encoder m_rightEncoder;
	/*
	 * private final SlewRateLimiter limiter1 = new SlewRateLimiter(5); private
	 * final SlewRateLimiter limiter2 = new SlewRateLimiter(5);
	 */

	public DrivetrainSys() {
		m_leftEncoder = new Encoder(0,1);
		m_rightEncoder = new Encoder(2,3);

		this.navx = new AHRS(Port.kMXP);
		this.frontLeft = new WPI_VictorSPX(Constants.FRONT_LEFT_DRIVE_CAN);
		frontLeft.configFactoryDefault();
		frontLeft.setNeutralMode(NeutralMode.Brake);

		this.frontRight = new WPI_VictorSPX(Constants.FRONT_RIGHT_DRIVE_CAN);
		frontRight.configFactoryDefault();
		frontRight.setNeutralMode(NeutralMode.Brake);

		this.backLeft = new WPI_VictorSPX(Constants.BACK_LEFT_DRIVE_CAN);
		backLeft.configFactoryDefault();
		backLeft.setNeutralMode(NeutralMode.Brake);
		backLeft.follow(frontLeft);

		this.backRight = new WPI_VictorSPX(Constants.BACK_RIGHT_DRIVE_CAN);
		backRight.configFactoryDefault();
		backRight.setNeutralMode(NeutralMode.Brake);
		backRight.follow(frontRight);

		this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
		this.drive = new DifferentialDrive(frontLeft, frontRight);
		drive.setSafetyEnabled(false);

	}

	public void setNeutralMode(NeutralMode mode) {
		frontLeft.setNeutralMode(mode);
		backLeft.setNeutralMode(mode);
		frontRight.setNeutralMode(mode);
		backRight.setNeutralMode(mode);
	}
	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		odometry.resetPosition(pose, new Rotation2d());
	  }
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
	  }

	  public double getAverageEncoderDistance() {
		return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
	  }
	public Pose2d getCurrentPose() {
		return odometry.getPoseMeters();
	}
	public Encoder getLeftEncoder() {
		return m_leftEncoder;
	  }
	  public Encoder getRightEncoder() {
		return m_rightEncoder;
	  }
	public void resetEncoders() {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
	  }
	public void arcadeDrive(double speed, double rotation) {
		drive.arcadeDrive(speed, rotation);
	}
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		frontLeft.setVoltage(leftVolts);
		frontRight.setVoltage(-rightVolts);
		drive.feed();
	  }

	public void stop() {
		drive.stopMotor();
	}

	public double getYaw() {
		return -navx.getYaw();
	}

	@Override
	public void periodic() {
	}
}
