package frc.robot.subsystems.climber;

import frc.lib.util.LynkMotor;
import frc.lib.util.LynkSubsystem;

public class Climber extends LynkSubsystem<Climber> {
    private static Climber instance;
    private final LynkMotor motor;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public Climber() {
        super("Climber");

        assert(instance == null) : "Climber instance already exists";
        instance = this;

        motor = addMotor(ClimberConstants.getMotorConfig());

        motor.stopMotor();
    }
}