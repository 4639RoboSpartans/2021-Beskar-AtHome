/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 4639. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package command;

/**
 * A command that prints a string when initialized.
 */
public class PrintCommand extends InstantCommand {
	/**
	 * Creates a new a PrintCommand.
	 *
	 * @param message
	 *            the message to print
	 */
	public PrintCommand(String message) {
		super(() -> System.out.println(message));
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
