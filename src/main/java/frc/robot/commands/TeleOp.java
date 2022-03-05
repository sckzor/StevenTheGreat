package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TeleOp extends CommandBase {

    private final DriveTrain m_driveTrain;
    private final Supplier<Double> m_xSpdFunction, m_ySpdFunction, m_turningSpdFunction;
    private final Supplier<Boolean> m_fieldOrientedFunction;
    private final SlewRateLimiter m_xLimiter, m_yLimiter, m_turningLimiter;

    public TeleOp(DriveTrain driveTrain,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        m_driveTrain = driveTrain;
        m_xSpdFunction = xSpdFunction;
        m_ySpdFunction = ySpdFunction;
        m_turningSpdFunction = turningSpdFunction;
        m_fieldOrientedFunction = fieldOrientedFunction;
        m_xLimiter = new SlewRateLimiter(Constants.DriveTrain.TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        m_yLimiter = new SlewRateLimiter(Constants.DriveTrain.TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        m_turningLimiter = new SlewRateLimiter(Constants.DriveTrain.TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        addRequirements(m_driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = m_xSpdFunction.get();
        double ySpeed = m_ySpdFunction.get();
        double turningSpeed = m_turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.TeleOp.DEADZONE ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.TeleOp.DEADZONE ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.TeleOp.DEADZONE ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = m_xLimiter.calculate(xSpeed) * Constants.DriveTrain.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        ySpeed = m_yLimiter.calculate(ySpeed) * Constants.DriveTrain.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        turningSpeed = m_turningLimiter.calculate(turningSpeed)
                * Constants.DriveTrain.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (m_fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, m_driveTrain.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.DriveTrain.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        m_driveTrain.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}