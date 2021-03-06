package org.usfirst.frc.team1322.robot.subsystems;

import org.usfirst.frc.team1322.robot.RobotMap;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;

public class SwerveDriveSubsystem extends HolonomicDrivetrain {
	private static final double WHEELBASE = 22.75;
	private static final double TRACKWIDTH = 22;
	private static final double RATIO = Math.sqrt(Math.pow(WHEELBASE, 2) + Math.pow(TRACKWIDTH, 2));

	/*
	 * 0 is Front Right
	 * 1 is Front Left
	 * 2 is Back Left
	 * 3 is Back Right
	 */
	private SwerveDriveModule[] mSwerveModules = new SwerveDriveModule[] {                            
		new SwerveDriveModule(0, new CANSparkMax(RobotMap.rrr, MotorType.kBrushless), new CANSparkMax(RobotMap.rrd, MotorType.kBrushless), 0), //real:390 practice: 212
		new SwerveDriveModule(1, new CANSparkMax(RobotMap.lrr, MotorType.kBrushless), new CANSparkMax(RobotMap.lrd, MotorType.kBrushless), 0), //real:293 practice: 59
		new SwerveDriveModule(2, new CANSparkMax(RobotMap.lfr, MotorType.kBrushless), new CANSparkMax(RobotMap.lfd, MotorType.kBrushless), 0), //real:298 practice: 56
		new SwerveDriveModule(3, new CANSparkMax(RobotMap.rfr, MotorType.kBrushless), new CANSparkMax(RobotMap.rfd, MotorType.kBrushless), 0)  //real: 355 practice: 190
	};

	private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);

	public SwerveDriveSubsystem() {
		zeroGyro(); 

		mSwerveModules[0].getDriveMotor().setInverted(true); //real: true
		mSwerveModules[1].getDriveMotor().setInverted(false); //real: false
		mSwerveModules[2].getDriveMotor().setInverted(false); //real: false
		mSwerveModules[3].getDriveMotor().setInverted(true); //real: true

		mSwerveModules[0].resetEncoder();
		for(int i = 0; i < 4; i++) {
			mSwerveModules[i].getDriveMotor().setIdleMode(IdleMode.kBrake);
		}
		
	}

	public AHRS getNavX() {
		return mNavX;
	}

	public double getGyroAngle() {
		return (mNavX.getAngle() - getAdjustmentAngle());
	}

	public double getGyroRate() {
		return mNavX.getRate();
	}

	public double getRawGyroAngle() {
		return mNavX.getAngle();
	}

	public double getYaw()
	{
		return mNavX.getAngle();
	}

	public SwerveDriveModule getSwerveModule(int i) {
		return mSwerveModules[i];
	}

	@Override
	public void holonomicDrive(double forward, double strafe, double rotation) {
		boolean isRotate = Math.abs(rotation) > .1;

		forward *= getSpeedMultiplier();
		strafe *= getSpeedMultiplier();
		/*
		if (isFieldOriented()) {
			double angleRad = Math.toRadians(getGyroAngle());
			double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad);
			strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
			forward = temp;
		}
		*/
		
		double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
		double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
		double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
		double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

		double[] angles = new double[]{
				Math.atan2(b, c) * 180 / Math.PI,
				Math.atan2(b, d) * 180 / Math.PI,
				Math.atan2(a, d) * 180 / Math.PI,
				Math.atan2(a, c) * 180 / Math.PI
		};

		double[] speeds = new double[]{
				Math.sqrt(b * b + c * c),
				Math.sqrt(b * b + d * d),
				Math.sqrt(a * a + d * d),
				Math.sqrt(a * a + c * c)
		};

		double max = speeds[0];

		// Limit Speeds
		for (double speed : speeds) { 
			if (speed > max) {
				max = speed;
			}
		}

		for (int i = 0; i < 4; i++) {
			if (Math.abs(forward) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(rotation) > 0.05) {
				mSwerveModules[i].setTargetAngle(angles[i] + 180, isRotate);
			} else {
				mSwerveModules[i].setTargetAngle(mSwerveModules[i].getTargetAngle(), isRotate);
			}
			mSwerveModules[i].setTargetSpeed(speeds[i]);
		}
	}

	@Override
	public void stopDriveMotors() {
		for (SwerveDriveModule module : mSwerveModules) {
			module.setTargetSpeed(0);
		}
	}

	public void resetAllEncoders() {
		for(int i = 0; i < 4; i++) {
			mSwerveModules[i].resetEncoder();
		}
	}

	public void resetAngleEncoders() {
		for(int i = 0; i < 4; i++) {
			mSwerveModules[i].resetAngleEncoder();
		}
	}

	public void driveForwardDistance(double targetPos, double angle, double speed){
		double angleError = ((angle - mNavX.getYaw()) / 180)*10;

		angleError = Math.min(angleError, 1);
		angleError = Math.max(angleError, -1);
		holonomicDrive(speed, 0, 0);
	}

	public void driveSidewaysDistance(double targetPos, double angle, double speed) {
		double angleError = ((angle - mNavX.getYaw()) / 180)*10;
		angleError = Math.min(angleError, 1);
		angleError = Math.max(angleError, -1);
		holonomicDrive(0, speed, angleError);
	}

	public double calculateErrPos(double d1) {
		return d1 - mSwerveModules[0].getDriveDistance();
	}

	public void commandDrivesToPos(double encPos) {
		for (int i = 0; i < 4; i++) {
			mSwerveModules[i].setPosByEnc(0);
		}
	}

}
