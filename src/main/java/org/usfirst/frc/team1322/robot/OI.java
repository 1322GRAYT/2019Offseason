package org.usfirst.frc.team1322.robot;

import org.usfirst.frc.team1322.robot.commands.*;
import org.usfirst.frc.team1322.robot.input.IGamepad;
import org.usfirst.frc.team1322.robot.input.XboxGamepad;
import org.usfirst.frc.team1322.robot.input.DPadButton.Direction;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private IGamepad mController = new XboxGamepad(0);
	private IGamepad climbController = new XboxGamepad(1);

	private Robot mRobot;

	public OI(Robot robot) {
		mRobot = robot;
	}

	public void registerControls() { 		
		mController.getYButton().whenPressed(new ZeroNavX());

		mController.getStartButton().whenPressed(new AdjustSpeedMultiplier(.3));
		mController.getBackButton().whenPressed(new AdjustSpeedMultiplier(.825));

		
		mController.getDPadButton(Direction.RIGHT).whenPressed(new AdjustFieldOrientedAngleCommand(mRobot.getDrivetrain(), true));
		mController.getDPadButton(Direction.LEFT).whenPressed(new AdjustFieldOrientedAngleCommand(mRobot.getDrivetrain(), false));
		
	}

	public IGamepad getController() {
		return mController;
	}

	public IGamepad getClimberController() {
		return climbController;
	}
}
