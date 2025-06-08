package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;



public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkAbsoluteEncoder turningEncoder;

    private final PIDController turningPIDController;
    
    @SuppressWarnings("unused")
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();
    SwerveModuleState state;
    /* ---------- sim ---------- */
    private SparkMaxSim driveSim, turnSim;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, 
                       boolean turningMotorReversed, boolean absoluteEncoderReversed,
                       double absoluteEncoderOffset) {
        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveConfig
            .inverted(driveMotorReversed)
            .idleMode(IdleMode.kBrake);
        driveConfig.encoder
            .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
            .velocityConversionFactor(12312);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0.0, 0.0);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turningConfig
            .inverted(turningMotorReversed)
            .idleMode(IdleMode.kBrake)
            .absoluteEncoder.zeroOffset(absoluteEncoderOffsetRad);
        turningConfig.encoder
            .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
            .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(0.1, 0.0, 0.0);


        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();      
    
        turningEncoder = turningMotor.getAbsoluteEncoder();     
      
        turningPIDController = new PIDController(ModuleConstants.kPModuleTurningController, 0, 0);
        turningPIDController.enableContinuousInput(0, 2*Math.PI);
        
        resetEncoders();

        if (RobotBase.isSimulation()) {
            driveSim = new SparkMaxSim(driveMotor, DCMotor.getNEO(1));
            turnSim  = new SparkMaxSim(turningMotor, DCMotor.getNEO(1));
        }
    }


    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }
    
    
    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }
    
    /**  Returns true wheel state (speed in m/s, angle in radians) */
    public SwerveModuleState getState() {
        double velocity =  driveEncoder.getVelocity()*ModuleConstants.kDriveEncoderRPM2MeterPerSec;

        return new SwerveModuleState(
            velocity,
            Rotation2d.fromRadians(turningEncoder.getPosition()));
    }

    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }
    
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        Rotation2d currentPos = Rotation2d.fromRadians(getTurningPosition());
        state.optimize(currentPos);
        state.cosineScale(currentPos);
        this.state = state;
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
    }
    
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    /* ---------------- simulation hook ---------------- */
    public void updateSim(double dtSeconds) {
        if (!RobotBase.isSimulation()) return;
    
        /* ---------- steer motor physics ---------- */
        double steerDuty = turningMotor.get();
        if (Math.abs(steerDuty) < 0.015) steerDuty = 0.0;
    
        double steerMotorRpm  = steerDuty * SimConstants.kNeoFreeRpm;
        turnSim.iterate(steerMotorRpm, 12.0, dtSeconds);                   // REV docs show iterate(ω, V, dt) :contentReference[oaicite:0]{index=0}
    
        /* ---------- drive motor physics ---------- */
        double driveDuty = driveMotor.get();
        if (Math.abs(driveDuty) < 0.02) driveDuty = 0.0;
    
        double driveMotorRpm = driveDuty * SimConstants.kNeoFreeRpm;
        driveSim.iterate(driveMotorRpm, 12.0, dtSeconds);
    }
}
