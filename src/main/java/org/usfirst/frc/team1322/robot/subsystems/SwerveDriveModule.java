package org.usfirst.frc.team1322.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SensorType;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1322.robot.commands.SwerveModuleCommand;
 

public class SwerveDriveModule extends Subsystem {
    // One rotation of the caddy, in encoder counts. Gear Ration is 19:1, so 19.5 is a rough estimate makes sense.
    private double oneRotation = 19.5; 
    private double mLastTargetAngle = 0;
    private double mStallTimeBegin = 0;
    private boolean negateDrivePower = false;

	private final int mModuleNumber;

	private final double mZeroOffset;

    private final CANEncoder mDriveEncoder;
    private final CANEncoder mAngleEncoder;
	private final CANSparkMax mAngleMotor;
    private final CANSparkMax mDriveMotor;
    
    private double zeroPos;

	public SwerveDriveModule(int moduleNumber, CANSparkMax angleMotor, CANSparkMax driveMotor, double zeroOffset) {
		mModuleNumber = moduleNumber;

		mAngleMotor = angleMotor;
        mDriveMotor = driveMotor;
        mDriveEncoder = mDriveMotor.getEncoder();
        mAngleEncoder = mAngleMotor.getEncoder();

        mZeroOffset = zeroOffset;
        zeroPos = mDriveEncoder.getPosition();

		// PID Controller
		CANPIDController m_pidController = angleMotor.getPIDController();

		// PID coefficients
		// TODO: Tune! These are ripped directly off of REV's Example project
		double kP = 0.1; 
		double kI = 1e-4;
		double kD = 1; 
		double kIz = 0; 
		double kFF = 0; 
		double kMaxOutput = 1; 
		double kMinOutput = -1;
	
		// set PID coefficients
		m_pidController.setP(kP);
		m_pidController.setI(kI);
		m_pidController.setD(kD);
		m_pidController.setIZone(kIz);
		m_pidController.setFF(kFF);
		m_pidController.setOutputRange(kMinOutput, kMaxOutput);
           
        // Set Idle Mode
        driveMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setIdleMode(IdleMode.kBrake);

        // Set Sensor Type
        angleMotor.setParameter(ConfigParameter.kSensorType, SensorType.kHallSensor.value);

        // Set amperage limits
        driveMotor.setSmartCurrentLimit(15);
        driveMotor.setSecondaryCurrentLimit(15,0);
        driveMotor.setCANTimeout(0);
        
        angleMotor.setSmartCurrentLimit(15);
        angleMotor.setSecondaryCurrentLimit(15,0);
        angleMotor.setCANTimeout(0);	     
	}
	
    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SwerveModuleCommand(this));
    }

    public CANSparkMax getAngleMotor() {
        return mAngleMotor;
    }

    /**
     * Get the current angle of the swerve module
     *
     * @return An angle in the range [0, 360)
     */
    public double getCurrentAngle() {
        double angle = mAngleEncoder.getPosition() * (360.0 / oneRotation);
        angle -= mZeroOffset;
        angle %= 360;
        if (angle < 0) angle += 360;

        return angle;
    }

  
    public CANSparkMax getDriveMotor() {
        return mDriveMotor;
    }

    public void robotDisabledInit() {
        mStallTimeBegin = Long.MAX_VALUE;
    }

    /**
     * This calculates the best way to get the enclosure to the desired position
     * @param targetAngle desired position
     */
    public void setTargetAngle(double targetAngle, boolean isRotate) {
        // Disable reverse if rotate because otherwise big big problems
        if(isRotate) negateDrivePower = false; 

        // It's ok beause smart dashboard is dumb
        SmartDashboard.putNumber("Module Current Angle (Enc Counts) " + mModuleNumber, mAngleEncoder.getPosition());
        mLastTargetAngle = targetAngle;

        // Calculate remainder after dividing by 360 degrees
        targetAngle %= 360; 
        
        // Get Current Position, then math it to be out of 360 degrees
        double currentAngle = mAngleEncoder.getPosition() * (360.0 / oneRotation);

        // Calculate Posotion to be a percent of 1
        double currentPercent = currentAngle / 360; // Percentage of angle out of 360
        double targetPercent = targetAngle / 360; // Percentage of angle out of 360

        // Print to SmartDashboard
        SmartDashboard.putNumber("Module Current Angle (Degrees) " + mModuleNumber, currentAngle);
        SmartDashboard.putNumber("Module Current Angle (Percent/360) " + mModuleNumber, currentPercent);

        // Calculate remainder after dividing by 360 degrees
        //double currentAngleMod = currentAngle;// % 360;

        double delta = currentPercent - targetPercent;
        SmartDashboard.putNumber("Module Delta (Perc/360) " + mModuleNumber, delta);

        // If drive power is negated, then we can subtract .5 from the ABS of the delta
        double deltaSubtraction = (negateDrivePower) ? .5 : 0;

        if(deltaSubtraction < .5) System.out.println(deltaSubtraction);

        if(Math.abs(delta) - deltaSubtraction > .15 && !isRotate) { // Invert angle, reverse motors
            System.out.println("Delta is greater, Reversing " + (Math.abs(delta) - deltaSubtraction));
            double first = 1 - targetPercent;
            double second = (currentPercent > .5) ? 1 - first : -1 * (1 - first);
            targetPercent = second;
            negateDrivePower = !negateDrivePower;
        }

        // This is angle compesation for when the drive power is reversed
        if (negateDrivePower) {
            if(targetPercent > .5) targetPercent -= .5;
            else targetPercent += .5;
        }

        // Prevent the caddy from spinning more than 1 rotation. 
        // This prevents us from having to invert motors. However, this
        // is slower, because we will have to rotate the caddy furthur to do what we want.
        double targetEncoderCount = targetPercent * oneRotation;
        
        //SmartDashboard.putNumber("Commanded Angle for Module (Enc Counts) " + mModuleNumber, targetEncoderCount);
        setPosByEnc(targetEncoderCount);
    }

    public void setPosByEnc(double pos) {
        mAngleMotor.getPIDController().setReference(pos, ControlType.kPosition);
    }

    public void setTargetSpeed(double speed) {
        mDriveMotor.set((negateDrivePower ? -1 : 1) * speed);
    }

    public void resetEncoder() {
        zeroPos = mDriveEncoder.getPosition(); 
    }

    public void resetAngleEncoder() {
        mAngleEncoder.setPosition(0);
    }

    public double getEncoderDiff() {
        return mDriveEncoder.getPosition() - zeroPos;
    }

    public double getTargetAngle() {
    	return mLastTargetAngle;
    }

    public double encoderTicksToInches(double ticks) {
         return ticks / 35.6;
    }

    public int inchesToEncoderTicks(double inches) {
         return (int) Math.round(inches * 35.6);
    }

    public double getInches() {
        return encoderTicksToInches(mDriveEncoder.getPosition());
    }

    public double getDriveDistance() { 
        double ticks = mDriveEncoder.getPosition();
        return encoderTicksToInches(ticks);
    }
}
