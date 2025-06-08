package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveRobot extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftAbsoluteEncoderReversed,
            DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightAbsoluteEncoderReversed,
            DriveConstants.kFrontRightAbsoluteEncoderOffsetRad);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftAbsoluteEncoderReversed,
            DriveConstants.kBackLeftAbsoluteEncoderOffsetRad);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightAbsoluteEncoderReversed,
            DriveConstants.kBackRightAbsoluteEncoderOffsetRad);

    private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    private final SwerveDrivePoseEstimator poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getRotation2d(),
            getModulePositions(),            // helper shown below
            new Pose2d()                     // start at (0,0,0)
        );  
    private final Field2d field = new Field2d();
    private Pose2d simPose = new Pose2d();
    /* --- SIMULATION‑ONLY OBJECTS --- */
    private final ADIS16470_IMUSim gyroSim = new ADIS16470_IMUSim(gyro);  
    public SwerveRobot() {
        SmartDashboard.putData("Field", field);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return (gyro.getAngle()%360.0);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    /* ---------- helpers ---------- */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),  frontRight.getState(),
            backLeft.getState(),   backRight.getState()
        };
    }
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),  frontRight.getPosition(),
            backLeft.getPosition(),   backRight.getPosition()
        };
    }
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }
    private double[] flattenStates() {        // for NT
        SwerveModuleState[] s = getModuleStates();
        return new double[] {
            s[0].speedMetersPerSecond, s[0].angle.getRadians(),
            s[1].speedMetersPerSecond, s[1].angle.getRadians(),
            s[2].speedMetersPerSecond, s[2].angle.getRadians(),
            s[3].speedMetersPerSecond, s[3].angle.getRadians()
        };
    }
    /* ---------- every 20 ms on real robot & sim ---------- */
    @Override
    public void periodic() {
        poseEstimator.update(getRotation2d(), getModulePositions());
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        // publish to SmartDashboard / NetworkTables
        ChassisSpeeds cs = getChassisSpeeds();
        SmartDashboard.putNumberArray("SwerveModuleStates", flattenStates());   // FL, FR, BL, BR
        SmartDashboard.putNumber("HeadingDeg", getHeading());
        SmartDashboard.putNumber("Chassis vx (m/s)",  cs.vxMetersPerSecond);
        SmartDashboard.putNumber("Chassis vy (m/s)",  cs.vyMetersPerSecond);
        SmartDashboard.putNumber("Chassis ω  (rad/s)",cs.omegaRadiansPerSecond);
        SmartDashboard.putData("Field", field);      // keeps the widget present
    }

/* ---------- sim‑only step ---------- */
    @Override
    public void simulationPeriodic() {
        final double dt = 0.02;  // TimedRobot default period

        // advance each module’s motor/encoder physics
        frontLeft.updateSim(dt);
        frontRight.updateSim(dt);
        backLeft.updateSim(dt);
        backRight.updateSim(dt);


        // integrate chassis speeds for a ground‑truth pose (optional but nice)
        ChassisSpeeds cs = getChassisSpeeds();
        simPose = simPose.exp(new Twist2d(
            cs.vxMetersPerSecond * dt,
            cs.vyMetersPerSecond * dt,
            cs.omegaRadiansPerSecond * dt));
        field.setRobotPose(simPose);
        gyroSim.setGyroAngleZ(simPose.getRotation().getDegrees());
    }
}