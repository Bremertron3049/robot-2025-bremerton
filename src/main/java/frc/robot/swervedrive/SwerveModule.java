package frc.robot.swervedrive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private SparkMax driveMotor, angleMotor;
    private RelativeEncoder driveEncoder;
    private SparkMaxConfig driveConfig = new SparkMaxConfig();


    private CANcoder angleEncoder;
    //private boolean reversedAngleEncoder;
    private double offsetAngleEncoder;

    private PIDController anglePID;

    public SwerveModule(int driveMotor, int angleMotor, int angleEncoder, double offsetAngleEncoder, boolean driveMotorReversed, boolean angleMotorReversed, boolean angleEncoderReversed){

        // Declare SparkMax motors
        this.driveMotor = new SparkMax(driveMotor, MotorType.kBrushless);
        this.angleMotor = new SparkMax(angleMotor, MotorType.kBrushless);

        // Drive motor and encoder configuration
        driveConfig
            .inverted(driveMotorReversed);

        // Configuring motors and encoders
        this.driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        this.angleMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        // Capturing drive motor encoder
        driveEncoder = this.driveMotor.getEncoder();

        // Declare Absolute Encoder
        this.angleEncoder = new CANcoder(angleEncoder);
        this.offsetAngleEncoder = offsetAngleEncoder;

        // Declare angle motor PIDController 
        anglePID = new PIDController(SwerveConstants.ANGLE_KP, 0, 0);
        anglePID.enableContinuousInput(0, 2 * Math.PI);

        resetEncoders();
    }

    // Sets driveEncoder to position zero.
    // angleEncoder is not included, as it is an absolute encoder and we don't want to reset it.
    public void resetEncoders(){
        driveEncoder.setPosition(0);
    }

    //UNIT: Radians
    public double getDrivePosition(){
        return (2 * Math.PI) * (driveEncoder.getPosition() * SwerveConstants.DRIVE_M_GEAR_RATIO);

        // Prior version, really shouldn't work but it does so here incase needed.
        // return (2 * Math.PI) * (driveEncoder.getPosition() / SwerveConstants.DRIVE_M_GEAR_RATIO);
    }

    //UNIT: Radians
    public double getAnglePosition(){
        //.getAbsolutePosition returns a 0-1 value, making the conversion to radians easy.
        return (2 * Math.PI) * angleEncoder.getAbsolutePosition().getValueAsDouble() - offsetAngleEncoder;
    }

    //UNIT: Meters per Second
    public double getDriveVelocity(){
        return driveEncoder.getVelocity() * SwerveConstants.DRIVE_E_RPM2MPS;
    }

    //UNIT: Meters per Second
    public double getAngleVelocity(){
        return angleEncoder.getVelocity().getValueAsDouble() * SwerveConstants.ANGLE_E_RPM2MPS;
    }

    //Returns the current SwerveModuleState.
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            getDriveVelocity(),
            Rotation2d.fromRadians(getAnglePosition())
        );
    }

    //Sets the desired SwerveModuleState of the SwerveModule.
    public void setState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.01){ // Deadband.
            stopModule();
            return;
        }

        state.optimize(getState().angle); // Optimizes the state given the current state's angle, allowing reverse positioning.

        angleMotor.set( // Sets angleMotor; Handled through PID for continuous input.
            anglePID.calculate(getAnglePosition(), state.angle.getRadians())
        );

        driveMotor.set(state.speedMetersPerSecond / SwerveConstants.ROBOT_MAX_SPEED); // Sets driveMotor; Input normalized between -1 and 1.
    }

    //Stops both motors.
    public void stopModule(){
        driveMotor.set(0);
        angleMotor.set(0);
    }

    //Returns the SwerveModulePosition of this SwerveModule, usually for odometry.
    //driveEncoder.getPosition is negative in order to (if I recall correctly) flip the axis it moves on in Glass field view.
    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(
            -driveEncoder.getPosition() * (SwerveConstants.WHEEL_CIRCUMFERENCE_METERS / SwerveConstants.DRIVE_M_GEAR_RATIO),
            Rotation2d.fromRadians(getAnglePosition())
        );
    }


    


}
