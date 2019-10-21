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
    private double oneRotation = 19.5; // One rotation of the caddy. Gear Ration is 19:1, so 19.5 is a rough estimate makes sense.
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
    public void setTargetAngle(double targetAngle) {
        mLastTargetAngle = targetAngle;

        // Calculate remainder after dividing by 360 degrees, then print to dashboard
        targetAngle %= 360; 
        SmartDashboard.putNumber("Module Target Angle (Degrees) " + mModuleNumber, targetAngle);
        
        // Add Offset
        targetAngle += mZeroOffset;

        // Get Current Position, then math it to be out of 360 degrees. Then Print to dashboard
        double currentAngle = mAngleEncoder.getPosition() * (360.0 / oneRotation);
        SmartDashboard.putNumber("Module Current Angle (Degrees) " + mModuleNumber, currentAngle);

        // Calculate remainder after dividing by 360 degrees
        double currentAngleMod = currentAngle % 360;

        boolean doItSmart = false;
        // This would calculate the fastest way to get the enclosure where it needs to go.
        // Unfortuntely for us, we can't use most of this becasue our drive motors are connected
        // Thus, we can't switch the direction of one motor without switching the direction of both,
        // We must use the full rotation of both our enclosures in order to prevent destroying our
        // Gearboxes. Sorry!
        if (doItSmart) {
            // No Negative Degrees
            if (currentAngleMod < 0) currentAngleMod += 360;

            // Calculate Difference from where we are to where we need to be
            double delta = currentAngleMod - targetAngle;

            // Calculate the fastest way to get to our desired position
            if (delta > 180) { // If we are more than 180 degrees off from target, add 360 degrees
                targetAngle += 360;
            } else if (delta < -180) { // If we're less than -180 degrees off of target, subtract 360 degrees
                targetAngle -= 360;
            }

            // Calculate direction motor must travel based on the way the caddy is facing
            delta = currentAngleMod - targetAngle;
            
            //If the difference between where it is and where it needs to be is greater than 90 degrees
            if (Math.abs(delta) > 90) {
                if (delta > 90) 
                    targetAngle += 180;
                else if (delta < -90)
                    targetAngle -= 180;
            } else {
                // If our drive motors wern't connected, we could reverse the motor's direction. 
                // Ours are connected, so we can't do that
                //mDriveMotor.setInverted(true); // We can't inverse the motors because they're connected :/
            }
        } else {
           // Prevent enclosure from rotating from 359 back to 0 again.
           //if(targetAngle > 180) {
           //     negateDrivePower = true;
           //     targetAngle /= 2;
           //} else {
           //    negateDrivePower = false;
           //}
        }
        
        // Calculate degrees back to Encoder Counts
        targetAngle += currentAngle - currentAngleMod;
        targetAngle *= oneRotation / 360.0;

        // Prevent the caddy from spinning more than 1 rotation. 
        // This prevents us from having to invert motors. However, this
        // is slower, because we will have to rotate the caddy furthur to do what we want.
        double targetEncoderCount = targetAngle % oneRotation;
        
        SmartDashboard.putNumber("Commanded Angle for Module (Enc Counts) " + mModuleNumber, targetEncoderCount);
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
