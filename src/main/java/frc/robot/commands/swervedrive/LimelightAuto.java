package frc.robot.commands.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;

public class LimelightAuto extends Command {

    private final SwerveSubsystem swerve;
    private final String limelightName = "limelight";

    private final DoubleSupplier vX;
    private final DoubleSupplier vY;

    // PID for Rotation: P=0.04 is a safe starting point for tx in degrees
    private final PIDController rotPID = new PIDController(0.04, 0.0, 0.003);

    public LimelightAuto(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;

        rotPID.setTolerance(1.0);
        
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double rotationSpeed;

        // 1. Check if Limelight sees a target
        if (LimelightHelpers.getTV(limelightName)) {
            double tx = LimelightHelpers.getTX(limelightName);
            // Calculate PID output based on horizontal offset (tx)
            rotationSpeed = -rotPID.calculate(tx, 0);
        } else {
            // No target? No rotation.
            rotationSpeed = 0;
        }

        // 2. Drive using Subsystem getters to solve the red line errors
        // vX and vY are scaled by the Max Velocity (m/s)
        // rotationSpeed is scaled by the Max Angular Velocity (rad/s)
        swerve.drive(
            new Translation2d(
                vX.getAsDouble() * swerve.getSwerveDrive().getMaximumChassisVelocity(), 
                vY.getAsDouble() * swerve.getSwerveDrive().getMaximumChassisVelocity()
            ),
            rotationSpeed * swerve.getSwerveDrive().getMaximumChassisAngularVelocity(),
            true // Field Centric
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        swerve.drive(new Translation2d(0, 0), 0, true);
    }
}