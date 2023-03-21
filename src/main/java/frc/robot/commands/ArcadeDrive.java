package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
 // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
    private final Drivetrain DT;
    private double turn;
    private double forward;
    private XboxController controller;

    public ArcadeDrive(Drivetrain drivetrain, double f, double t, XboxController controlla) {
        DT = drivetrain;
        turn = t;
        forward = f;
        controller = controlla;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        forward = controller.getLeftY();
        turn = controller.getRightX();
    }

    @Override
    public void execute() {
        forward = Constants.ControlConstants.maxArcadeSpeed * controller.getLeftY();
        turn = Constants.ControlConstants.maxArcadeSpeed * controller.getRightX();

        DT.controllerArcadeDrive(forward, turn, controller);

            /*
            * SmartDashboard.putNumber("", volts)
            x4
            */
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
