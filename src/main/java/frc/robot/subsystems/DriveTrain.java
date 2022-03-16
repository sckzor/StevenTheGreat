package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    private final SwerveModule m_bird = new SwerveModule(
            Constants.DriveTrain.BIRD_DRIVE_MOTOR_ID,
            Constants.DriveTrain.BIRD_TURN_MOTOR_ID,
            Constants.DriveTrain.BIRD_TURN_MOTOR_ENCODER_OFFSET,
            Constants.DriveTrain.BIRD_DRIVE_MOTOR_REVERSED,
            Constants.DriveTrain.BIRD_TURN_MOTOR_REVERSED);

    private final SwerveModule m_sushi = new SwerveModule(
            Constants.DriveTrain.SUSHI_DRIVE_MOTOR_ID,
            Constants.DriveTrain.SUSHI_TURN_MOTOR_ID,
            Constants.DriveTrain.SUSHI_TURN_MOTOR_ENCODER_OFFSET,
            Constants.DriveTrain.SUSHI_DRIVE_MOTOR_REVERSED,
            Constants.DriveTrain.SUSHI_TURN_MOTOR_REVERSED);

    private final SwerveModule m_horse = new SwerveModule(
            Constants.DriveTrain.HORSE_DRIVE_MOTOR_ID,
            Constants.DriveTrain.HORSE_TURN_MOTOR_ID,
            Constants.DriveTrain.HORSE_TURN_MOTOR_ENCODER_OFFSET,
            Constants.DriveTrain.HORSE_DRIVE_MOTOR_REVERSED,
            Constants.DriveTrain.HORSE_TURN_MOTOR_REVERSED);

    private final SwerveModule m_giraffe = new SwerveModule(
            Constants.DriveTrain.GIRAFFE_DRIVE_MOTOR_ID,
            Constants.DriveTrain.GIRAFFE_TURN_MOTOR_ID,
            Constants.DriveTrain.GIRAFFE_TURN_MOTOR_ENCODER_OFFSET,
            Constants.DriveTrain.GIRAFFE_DRIVE_MOTOR_REVERSED,
            Constants.DriveTrain.GIRAFFE_TURN_MOTOR_REVERSED);

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry m_odometer = new SwerveDriveOdometry(Constants.DriveTrain.SWERVE_DRIVE_KINEMATICS,
            new Rotation2d(0));

    public DriveTrain() {
        new Thread(() -> {
            try {
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return m_odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        m_odometer.update(getRotation2d(), m_horse.getState(), m_giraffe.getState(), m_sushi.getState(),
                 m_bird.getState());
    }

    public void stopModules() {
        m_horse.stop();
        m_giraffe.stop();
        m_sushi.stop();
        m_bird.stop();
    }

    public void testTurning() {
        m_horse.testTurning();
        m_giraffe.testTurning();
        m_sushi.testTurning();
        m_bird.testTurning();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveTrain.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        m_horse.setDesiredState(desiredStates[0]);
        m_giraffe.setDesiredState(desiredStates[1]);
        m_sushi.setDesiredState(desiredStates[2]);
        m_bird.setDesiredState(desiredStates[3]);
    }
}