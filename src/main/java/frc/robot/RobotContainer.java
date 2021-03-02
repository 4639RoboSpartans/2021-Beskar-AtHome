/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import static frc.robot.Constants.Buttons;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.ManualDriveCmd;
import frc.robot.commands.PushBallsCmd;
import frc.robot.commands.SpoolShooterCmd;
import frc.robot.commands.Turret90Cmd;
import frc.robot.subsystems.ClimberSys;
import frc.robot.subsystems.DrivetrainSys;
import frc.robot.subsystems.HopperSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.KickerSys;
import frc.robot.subsystems.ShooterSys;
import frc.robot.subsystems.ShroudSys;
import frc.robot.subsystems.TurretSys;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import command.Command;
import command.ExecuteEndCommand;
import command.InstantCommand;
import command.ParallelCommandGroup;
import command.RamseteCommand;
import command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final ClimberSys m_climber= new ClimberSys();
	public final DrivetrainSys m_drive= new DrivetrainSys();
	public final IntakeSys m_intake= new IntakeSys();
	private final HopperSys m_hopper= new HopperSys();
	private final ShooterSys m_shooter = new ShooterSys();
	private final KickerSys m_kicker= new KickerSys();
	private final TurretSys m_turret= new TurretSys();
	private final ShroudSys m_shroud = new ShroudSys();
	private static final double diameter = 6.00;
	private final OI m_oi= new OI();
	private final Compressor m_compressor= new Compressor();
	private int shroudPos = 0;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_compressor.setClosedLoopControl(true);

		// Climber commands removed. RIGHT STICK is now used for the Shroud
		// m_climber.setDefaultCommand(
		// new ExecuteEndCommand(() -> m_climber.setClimber(m_oi.getAxis(1,
		// Constants.Axes.RIGHT_STICK_Y)),
		// () -> m_climber.setClimber(0), m_climber));
		// m_climber.setPistons(DoubleSolenoid.Value.kReverse);

		m_drive.setDefaultCommand(new ManualDriveCmd(m_drive, m_oi));

		m_intake.setDefaultCommand(new ExecuteEndCommand(() -> {
			if (m_oi.getAxis(1, Constants.Axes.RIGHT_TRIGGER) > 0) {
				m_hopper.setHopper(0.0);
				m_intake.setIntake(0.5);
			} else if (m_oi.getAxis(1, Constants.Axes.LEFT_TRIGGER) > 0) {
				m_hopper.setHopper(0);
				m_intake.setIntake(-0.5);
				// } else if(m_oi.getButton(1, Constants.Buttons.RIGHT_BUMPER)) {
				// ///ksbflwglwrglgblwg
				// m_hopper.setHopper(0);
				// m_intake.setIntake(0);
				// }
			} else {
				m_hopper.setHopper(0);
				m_intake.setIntake(0);
			}
		}, () -> {
			m_hopper.setHopper(0);
			m_intake.setIntake(0);
		}, m_intake, m_hopper));

		m_turret.setDefaultCommand(
				new ExecuteEndCommand(() -> m_turret.setTurret(m_oi.getAxis(1, Constants.Axes.LEFT_STICK_X) * 0.5),
						() -> m_turret.setTurret(0), m_turret));

		// m_shroud.setDefaultCommand(
		// new ExecuteEndCommand(() -> m_shroud.setShroud(m_oi.getAxis(1,
		// Constants.Axes.RIGHT_STICK_Y) * 0.3),
		// () -> m_shroud.setShroud(0), m_shroud));
		configureButtonBindings();
		
		//SmartDashboard.putString("DB/String 8", "LEncoderVal: "+m_drive.m_leftEncoder.getDistance());
		//SmartDashboard.putString("DB/String 9", "REncoderVal: "+m_drive.m_rightEncoder.getDistance());
	}
	// if the shroud is set to a position less than 3, then this will increase the
	// position value by 1, and then return that position.
	private int goDown() {
		SmartDashboard.putString("DB/String 5", "GoUp: True");
		if (shroudPos < 3) {
			return ++this.shroudPos;
		} else
			return this.shroudPos;
	}

	// if the shroud is set to a position less than 4, then this will increase the
	// position value by 1, and then return that position.
	private int goUp() {
		SmartDashboard.putString("DB/String 5", "GoDown: True");
		if (shroudPos > 0) {
			return --this.shroudPos;
		} else
			return this.shroudPos;
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// Toggle Shroud Presets
		m_oi.getPovButton(1, 270).whenPressed(new InstantCommand(() -> m_shroud.setDesiredPosition(goUp()), m_shroud));
		m_oi.getPovButton(1, 90).whenPressed(new InstantCommand(() -> m_shroud.setDesiredPosition(goDown()), m_shroud));

		SmartDashboard.putString("DB/String 1", "RightPOV: " + m_oi.getPovButton(1, 90).get());
		SmartDashboard.putString("DB/String 2", "LeftPOV: " + m_oi.getPovButton(1, 270).get());
		SmartDashboard.putString("DB/String 3", "UpPOV:" + m_oi.getPovButton(1, 0).get());
		SmartDashboard.putString("DB/String 4", "DownPOV:" + m_oi.getPovButton(1, 180).get());
		// Bring intake up
		m_oi.getPovButton(1, 0)
				.whileHeld(new ExecuteEndCommand(() -> m_intake.setPivot(0.7), () -> m_intake.setPivot(0), m_intake));

		// Run Hopper In
		m_oi.getButton(1, Buttons.RIGHT_BUMPER).whileHeld(new InstantCommand(() -> m_hopper.setHopper(0.5), m_hopper));
		m_oi.getButton(1, Buttons.RIGHT_BUMPER).whileHeld(new InstantCommand(() -> m_intake.setIntake(0.5), m_intake));

		// Bring intake down
		m_oi.getPovButton(1, 180)
				.whileHeld(new ExecuteEndCommand(() -> m_intake.setPivot(-0.5), () -> m_intake.setPivot(0), m_intake));

		// Extend the climber pistons
		m_oi.getButton(1, Buttons.Y_BUTTON)
				.whileHeld(new InstantCommand(() -> m_climber.setPistons(DoubleSolenoid.Value.kForward), m_climber));
		m_oi.getButton(1, Buttons.A_BUTTON)
				.whileHeld(new InstantCommand(() -> m_climber.setPistons(DoubleSolenoid.Value.kReverse), m_climber));

		// Move kicker wheel back to clear ball and then spool the shooter
		m_oi.getButton(1, Buttons.X_BUTTON)
				.whileHeld(new ExecuteEndCommand(() -> m_kicker.setKicker(0), () -> m_kicker.setKicker(0), m_kicker)
						.withTimeout(0.1).andThen(new SpoolShooterCmd(m_shooter, m_kicker, 3800)));

		m_oi.getButton(1, Buttons.B_BUTTON)
				.whileHeld(new ExecuteEndCommand(() -> m_kicker.setKicker(-0.5), () -> m_kicker.setKicker(0), m_kicker)
						.withTimeout(0.1).andThen(new SpoolShooterCmd(m_shooter, m_kicker, 4300)));
		m_oi.getButton(1, Buttons.B_BUTTON).whileHeld(new InstantCommand(() -> m_hopper.setHopper(-0.5), m_hopper));
		m_oi.getButton(1, Buttons.B_BUTTON).whileHeld(new InstantCommand(() -> m_intake.setIntake(-0.5), m_intake));

		// Use the kicker to push the balls in
		m_oi.getButton(0, Buttons.X_BUTTON).whileHeld(new PushBallsCmd(m_hopper, m_intake, m_shooter));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command TestAuton(){
		return new ParallelCommandGroup(
			new ExecuteEndCommand(() -> m_intake.setIntake(0.5), () -> m_intake.setIntake(0), m_intake).withTimeout(3),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.5, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(3))
			.andThen(
			new ParallelCommandGroup(
			new ExecuteEndCommand(() -> m_intake.setIntake(0), () -> m_intake.setIntake(0), m_intake).withTimeout(3),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.5, -45), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(3)
			));
	}

	public Command originialAuton(){
		return new ParallelCommandGroup(
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(-0.5, 0), () -> m_drive.arcadeDrive(0, 0), m_drive)
					.withTimeout(1.5),
			new Turret90Cmd(m_turret), new WaitCommand(5))
					.andThen(new ParallelCommandGroup(new SpoolShooterCmd(m_shooter, m_kicker, 3800),
							new PushBallsCmd(m_hopper, m_intake, m_shooter)).withTimeout(7));
	}
	public Command getAutonomousCommand() {
		return TestAuton();
	}
	
	//setting up auton with pathweaver
	/*public Command TestAuton(){
		var autoVoltageConstraint = 
			new DifferentialDriveVoltageConstraint(
				Constants.DRIVETRAIN_FEED_FORWARD, 
					Constants.kDriveKinematics, 10);

		TrajectoryConfig config= 
			new TrajectoryConfig(Constants.MaxSpeed,
								Constants.MaxAccel)
									.setKinematics(Constants.kDriveKinematics)
										.addConstraint(autoVoltageConstraint);

		/*String trajJSON = "paths/Test.wpilib.json";
		Trajectory trajectory = new Trajectory();
		try{
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajJSON);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		}catch(IOException ex){
			DriverStation.reportError("Unable to open traj:"+ trajJSON, ex.getStackTrace());
		}
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0.0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(
				new Translation2d(1, 1),
				new Translation2d(2, -1)
			),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, new Rotation2d(0.0)),
			// Pass config
			config
		);

		RamseteCommand ramseteCommand = new RamseteCommand(
			trajectory, 
			m_drive::getCurrentPose, 
			new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
			new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka), 
			Constants.kDriveKinematics, 
			m_drive::getWheelSpeeds, 
			new PIDController(Constants.kPDriveVel,0,0), 
			new PIDController(Constants.kPDriveVel,0,0), 
			m_drive::tankDriveVolts, 
			m_drive);

			m_drive.resetOdometry(trajectory.getInitialPose());

			return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
	}*/


	public void setDriveNeutralMode(NeutralMode mode) {
		m_drive.setNeutralMode(mode);
	}
}
