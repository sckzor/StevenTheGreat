// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax m_driveMotor;
    private final TalonSRX m_turnMotor;
    private final RelativeEncoder m_driveEncoder;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed) {
        m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        m_driveMotor.setInverted(driveMotorReversed);
        m_driveEncoder = m_driveMotor.getEncoder();

        m_turnMotor = new TalonSRX(turnMotorId);
        m_turnMotor.setInverted(turnMotorReversed);
        m_turnMotor.setSensorPhase(turnMotorReversed);
        m_turnMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

        m_driveEncoder.setPositionConversionFactor(Constants.SwerveModule.DRIVE_ENCODER_ROTATIONS_TO_METERS);
        m_driveEncoder.setVelocityConversionFactor(Constants.SwerveModule.DRIVE_ENCODER_RPM_TO_METERS_PER_SECOND);
    }

    public double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return m_driveEncoder.getVelocity();
    }

    public double getTurningPosition() {
        return (m_turnMotor.getSelectedSensorPosition() / 4096) * 2 * Math.PI;
    }

    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        m_driveMotor.set(state.speedMetersPerSecond / Constants.DriveTrain.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        m_turnMotor.set(TalonSRXControlMode.Position, (state.angle.getRadians() / (Math.PI * 2)) * 4096);
    }

    public void stop() {
        m_driveMotor.set(0);
        m_turnMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
