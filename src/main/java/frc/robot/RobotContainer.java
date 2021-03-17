/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import java.io.*;
import java.util.*;

import static frc.robot.Constants.Buttons;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ManualDriveCmd;
import frc.robot.commands.PushBallsCmd;
import frc.robot.commands.SpoolShooterCmd;
import frc.robot.commands.Turret90Cmd;
import frc.robot.subsystems.ClimberSys;
import frc.robot.subsystems.DrivetrainSys;
import frc.robot.subsystems.HopperSys;
import frc.robot.subsystems.IntakeElevator;
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
	public final IntakeElevator m_intakePiv = new IntakeElevator();
	private final HopperSys m_hopper= new HopperSys();
	private final ShooterSys m_shooter = new ShooterSys();
	private final KickerSys m_kicker= new KickerSys();
	private final TurretSys m_turret= new TurretSys();
	private final ShroudSys m_shroud = new ShroudSys();
	private static final double diameter = 6.00;
	private final OI m_oi= new OI();
	private final Compressor m_compressor= new Compressor();
	private int shroudPos = 0;
	public double time1;
	public double time2;
	public double time3;
	public double time4;
	public double time5;
	public double time6;
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
				.whileHeld(new ExecuteEndCommand(() -> m_intakePiv.setPivot(0.7), () -> m_intakePiv.setPivot(0), m_intake));

		// Run Hopper In
		m_oi.getButton(1, Buttons.RIGHT_BUMPER).whileHeld(new InstantCommand(() -> m_hopper.setHopper(0.5), m_hopper));
		m_oi.getButton(1, Buttons.RIGHT_BUMPER).whileHeld(new InstantCommand(() -> m_intake.setIntake(0.5), m_intake));

		// Bring intake down
		m_oi.getPovButton(1, 180)
				.whileHeld(new ExecuteEndCommand(() -> m_intakePiv.setPivot(-0.5), () -> m_intakePiv.setPivot(0), m_intake));

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
	//distance for every rotation is 3.14*6
	//theoretic value for going certain distance is 0.04 at full power
	//turning circle radius is 19.799 in, circumference is 124.4 in.
	//to turn with both sides, each side will travel the wanted degrees: 45:15.55 90:31.1
	//timing for 45 degrees is: 0.009 at full speed
	//timing for 90 degress is: 0.018 at full speed
	public Command getAutonomousCommand() {
		return RedPath2();
	}

	public Command RedPath1(){//COMPLETE-PENDING RECORDING
		return new ParallelCommandGroup(//go forward
		
			new ExecuteEndCommand(()->m_intakePiv.setPivot(-0.7), ()->m_intakePiv.setPivot(0), m_intakePiv).withTimeout(0.8),
			new ExecuteEndCommand(() -> m_intake.setIntake(0.7), () -> m_intake.setIntake(0.7), m_intake).withTimeout(1.25),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.75, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(1.25))
			.andThen(//1.25
			new ParallelCommandGroup(//turn 45 deg right
			new ExecuteEndCommand(() -> m_intake.setIntake(0.7), () -> m_intake.setIntake(0.7), m_intake).withTimeout(0.12),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.5, 45), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0.12)
			))//0.12	
			.andThen(
			new ParallelCommandGroup(//go forward
			new ExecuteEndCommand(() -> m_intake.setIntake(0.7), () -> m_intake.setIntake(0.7), m_intake).withTimeout(0.7),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.75, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0.7)
			))//0.4

			.andThen(
			new ParallelCommandGroup(//turn 90 deg left
			new ExecuteEndCommand(() -> m_intake.setIntake(0.5), () -> m_intake.setIntake(0.5), m_intake).withTimeout(0.25),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.5, -45), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0.25)
			))//0.3
			.andThen(
			new ParallelCommandGroup(//go forward
			new ExecuteEndCommand(() -> m_intake.setIntake(0.7), () -> m_intake.setIntake(0.7), m_intake).withTimeout(1.32),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.75, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(1.32)))//1
			.andThen(
			new ParallelCommandGroup(//turn 45 deg right
			new ExecuteEndCommand(() -> m_intake.setIntake(0.7), () -> m_intake.setIntake(0), m_intake).withTimeout(0.30),//0.25
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.5, 45), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0.30)
			))
			.andThen(
			new ParallelCommandGroup(//go forward
			new ExecuteEndCommand(() -> m_intake.setIntake(0), () -> m_intake.setIntake(0), m_intake).withTimeout(1.2),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.75, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(1.2)
		));
	}
	public Command SlalomPath(){//INCOMPLETE
		time1 =SmartDashboard.getNumber("time1", 0);
		time2 =SmartDashboard.getNumber("time2", 0);
		time3 =SmartDashboard.getNumber("time3", 0);
		time4 =SmartDashboard.getNumber("time4", 0);
		//go forward
		return new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.75, 0), ()-> m_drive.arcadeDrive(0, 0),m_drive).withTimeout(time1)
		.andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()->m_drive.arcadeDrive(0, 0), m_drive).withTimeout(time2)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(time3)
		).andThen(
			//turn right
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 45), ()->m_drive.arcadeDrive(0, 0), m_drive).withTimeout(time4)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//turn right
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 45), ()-> m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//go around marker
			new ExecuteEndCommand(()->m_drive.curvatureDrive(0.5, -180, false), ()->m_drive.curvatureDrive(0, 0, false), m_drive).withTimeout(0)
		).andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//turn right
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5,45), ()->m_drive.arcadeDrive(0,0),m_drive).withTimeout(0)
		).andThen(
			//goForward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive). withTimeout(0)
		).andThen(
			//turn right
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 45), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75,0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		)
		;
	}
	public Command BouncePath(){//INCOMPLETE
		SmartDashboard.getNumber("time1", 0);
		SmartDashboard.getNumber("time2", 0);
		SmartDashboard.getNumber("time3", 0);
		SmartDashboard.getNumber("time4", 0);
		SmartDashboard.getNumber("time5", 0);
		SmartDashboard.getNumber("time6", 0);
		//go forward
		return new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75,0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(time1)
		.andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(time2)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(time3)
		).andThen(
			//go backward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(-0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(time4)
		)
		.andThen(
			//slightly turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(time5)
		).andThen(
			//go backwards
			new ExecuteEndCommand(()->m_drive.arcadeDrive(-0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(time6)
		).andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//go backwards
			new ExecuteEndCommand(()->m_drive.arcadeDrive(-0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//go forwards
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//turn slightly left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		).andThen(
			//go backwards
			new ExecuteEndCommand(()->m_drive.arcadeDrive(-0.75, 0), ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(0)
		);

	}
	public Command BarrelRollPath(){//INCOMPLETE
		SmartDashboard.getNumber("time1", 0);
		SmartDashboard.getNumber("time2", 0);
		SmartDashboard.getNumber("time3", 0);
		SmartDashboard.getNumber("time4", 0);
		SmartDashboard.getNumber("time5", 0);
		SmartDashboard.getNumber("time6", 0);
		//go forward
		return new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(time1)
		.andThen(
			//turn right
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 45), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(time2)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 0), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(time3)
		).andThen(
			//turn right
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 45), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(time4)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 0), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(time5)
		).andThen(
			//turn right
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 45), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(time6)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 0), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//turn right
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 45), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 0), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 0), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, 0), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//turn left
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.5, -45), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		).andThen(
			//go forward
			new ExecuteEndCommand(()->m_drive.arcadeDrive(0.75, 0), ()-> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0)
		)
		;
	}
/*	
	public Command BluePath1(){

	}
	public Command BluePath2(){

	}
	*/
	
	public Command RedPath2(){//COMPLETE-PENDING RECORDING
		return new ParallelCommandGroup(//go forward
			new ExecuteEndCommand(()->m_intakePiv.setPivot(-0.7), ()->m_intakePiv.setPivot(0), m_intakePiv).withTimeout(0.8),
			new ExecuteEndCommand(() -> m_intake.setIntake(0.7), () -> m_intake.setIntake(0), m_intake).withTimeout(1.25),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.75, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(1.25))
			.andThen(//1.65
			new ParallelCommandGroup(//turn 45 deg right
			new ExecuteEndCommand(() -> m_intake.setIntake(0.7), () -> m_intake.setIntake(0), m_intake).withTimeout(0.18),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.5, 45), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0.18)
			))//0.18	
			.andThen(
			new ParallelCommandGroup(//go forward
			new ExecuteEndCommand(() -> m_intake.setIntake(0.9), () -> m_intake.setIntake(0), m_intake).withTimeout(0.9),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.75, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0.9)
			))//0.82

			.andThen(
			new ParallelCommandGroup(//turn 90 deg left
			new ExecuteEndCommand(() -> m_intake.setIntake(0.5), () -> m_intake.setIntake(0), m_intake).withTimeout(0.25),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.5, -90), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0.25)
			))//0.3
			.andThen(
			new ParallelCommandGroup(//go forward
			new ExecuteEndCommand(() -> m_intake.setIntake(0.7), () -> m_intake.setIntake(0), m_intake).withTimeout(1.4),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.75, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(1.4)
			))//1
			.andThen(
			new ParallelCommandGroup(//turn 45 deg right
			new ExecuteEndCommand(() -> m_intake.setIntake(0.7), () -> m_intake.setIntake(0), m_intake).withTimeout(0.2),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.5, 45), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0.24)
			))
			.andThen(
			new ParallelCommandGroup(//go forward
			new ExecuteEndCommand(() -> m_intake.setIntake(0), () -> m_intake.setIntake(0), m_intake).withTimeout(0.8),
			new ExecuteEndCommand(() -> m_drive.arcadeDrive(0.75, 0), () -> m_drive.arcadeDrive(0, 0), m_drive).withTimeout(0.8)
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
	
	
	public void setDriveNeutralMode(NeutralMode mode) {
		m_drive.setNeutralMode(mode);
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
}
