package org.usfirst.frc.team1322.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	public static final int lfd = 6; // Left Front Drive
	public static final int lrd = 5; // Left Rear Drive
  
	public static final int rfd = 8; // Right Front Drive
	public static final int rrd = 2; // Right Rear Drive
  
	public static final int lfr = 7; // Left Front Rot
	public static final int lrr = 4; // Left Rear Rot
	public static final int rfr = 1; // Right Front Rot
	public static final int rrr = 3; // Right Rear Rot


	public static final int kTimeoutMs = 10;
	public static final int kPIDLoopIdx = 0;
}
