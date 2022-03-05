package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleOp;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {

    private final DriveTrain swerveSubsystem = new DriveTrain();

    private final Joystick driverJoytick = new Joystick(Constants.TeleOp.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new TeleOp(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(Constants.TeleOp.DRIVER_Y_AXIS),
                () -> driverJoytick.getRawAxis(Constants.TeleOp.DRIVER_X_AXIS),
                () -> driverJoytick.getRawAxis(Constants.TeleOp.DRIVER_ROT_AXIS),
                () -> !driverJoytick.getRawButton(Constants.TeleOp.DRIVER_FIELD_ORIENTED_BUTTON_INDEX)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    }

}