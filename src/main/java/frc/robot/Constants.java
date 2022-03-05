// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveModule {
        public static final double WHEEL_DIAMETER = 0.08;
        public static final double DRIVE_MOTOR_REDUCTION = 0.33;
        public static final double TURN_MOTOR_REDUCTION = 0.02;
        public static final double DRIVE_ENCODER_ROTATIONS_TO_METERS = DRIVE_MOTOR_REDUCTION * Math.PI * WHEEL_DIAMETER;
        public static final double TURN_ENCODER_ROTATIONS_TO_RADIANS = TURN_MOTOR_REDUCTION * 2 * Math.PI;
        public static final double DRIVE_ENCODER_RPM_TO_METERS_PER_SECOND = DRIVE_ENCODER_ROTATIONS_TO_METERS / 60;
        public static final double TURN_ENCODER_RPM_TO_RADIANS_PER_SECOND = TURN_ENCODER_ROTATIONS_TO_RADIANS / 60;
        public static final double TURNING_P = 0.5;
    }

    public static final class DriveTrain {
        // Horse to giraffe length
        public static final double TRACK_WIDTH = 0.53;
        
        // Sushi to horse length
        public static final double WHEEL_BASE = 0.525;

        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)
        );

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 1.0;
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 1.0 * 1.0 * Math.PI;

        public static final int GIRAFFE_DRIVE_MOTOR_ID = 2;
        public static final int HORSE_DRIVE_MOTOR_ID = 12;
        public static final int SUSHI_DRIVE_MOTOR_ID = 15;
        public static final int BIRD_DRIVE_MOTOR_ID = 1;

        public static final int GIRAFFE_TURN_MOTOR_ID = 3;
        public static final int HORSE_TURN_MOTOR_ID = 13;
        public static final int SUSHI_TURN_MOTOR_ID = 14;
        public static final int BIRD_TURN_MOTOR_ID = 20;
        
        public static final boolean GIRAFFE_DRIVE_MOTOR_REVERSED = false;
        public static final boolean HORSE_DRIVE_MOTOR_REVERSED = false;
        public static final boolean SUSHI_DRIVE_MOTOR_REVERSED = false;
        public static final boolean BIRD_DRIVE_MOTOR_REVERSED = false;
        
        public static final boolean GIRAFFE_TURN_MOTOR_REVERSED = false;
        public static final boolean HORSE_TURN_MOTOR_REVERSED = false;
        public static final boolean SUSHI_TURN_MOTOR_REVERSED = false;
        public static final boolean BIRD_TURN_MOTOR_REVERSED = false;
        
        public static final double TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 1;
        public static final double TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 1;
    }

    public static final class TeleOp { 
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int DRIVER_Y_AXIS = 1;
        public static final int DRIVER_X_AXIS = 0;
        public static final int DRIVER_ROT_AXIS = 4;
        public static final int DRIVER_FIELD_ORIENTED_BUTTON_INDEX = 1;

        public static final double DEADZONE = 0.05;
    }
}
