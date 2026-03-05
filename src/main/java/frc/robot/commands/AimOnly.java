package frc.robot.commands;

import frc.lib.util.LoggedCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveAlign;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AimOnly extends LoggedCommand {
    private final Swerve s_Swerve;
    private Supplier<Optional<Rotation2d>> autoAimSupplier = null;

    public AimOnly(Swerve s_Swerve, Supplier<Optional<Rotation2d>> autoAimSupplier) {
        super();

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.autoAimSupplier = autoAimSupplier;
    }

    @Override
    public void initialize() {
        super.initialize();
        SwerveAlign.errorReset();
    }

    @Override
    public void execute() {
        super.execute();

        double rotationVal = 0.0;
        Optional<Rotation2d> optAngle = autoAimSupplier.get();
        if (optAngle.isPresent()) {
            SwerveAlign.setTarget(optAngle.get()); // Constantly update target
            rotationVal = SwerveAlign.getSpeed();
        }
 
        /* Drive */
        s_Swerve.drive(
            new Translation2d(0.0, 0.0),
            rotationVal * SwerveConstants.maxAngularVelocity,
            false
        );
    }
}