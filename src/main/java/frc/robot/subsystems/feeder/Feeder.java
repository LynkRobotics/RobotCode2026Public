package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedCommands;
import frc.lib.util.LynkMotor;
import frc.lib.util.LynkSubsystem;
import frc.robot.Ports;

public class Feeder extends LynkSubsystem<Feeder> {
    private static Feeder instance;
    private final LynkMotor floorMotor;
    private final LynkMotor floorMotorB;
    private final LynkMotor feederMotor;

    public static Feeder getInstance() {
        if (instance == null) {
            instance = new Feeder();
        }
        return instance;
    }

    public Feeder() {
        super("Feeder");

        assert(instance == null) : "Feeder instance already exists";
        instance = this;

        floorMotor = addMotor(FeederConstants.getFloorMotorConfig());
        floorMotorB = addMotor("Floor B", Ports.FLOOR_B);
        feederMotor = addMotor(FeederConstants.getFeederMotorConfig());

        floorMotor.stopMotor();
        feederMotor.stopMotor();

        floorMotorB.setControl(new Follower(floorMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void feed() {
        // floorMotor.set(FeederConstants.floorSpeed);
        // feederMotor.set(FeederConstants.feederSpeed);
        feederMotor.setControl(new VoltageOut(12.0).withEnableFOC(true));
        floorMotor.setControl(new VoltageOut(12.0).withEnableFOC(true));
    }

    public void stop() {
        floorMotor.stopMotor();
        feederMotor.stopMotor();
    }

    public Command Feed() {
        return LoggedCommands.startEnd("Feed", this::feed, this::stop, this);
    }

    private void reverse() {
        feederMotor.setControl(new VoltageOut(-6.0).withEnableFOC(true));
        floorMotor.setControl(new VoltageOut(-6.0).withEnableFOC(true));
    }

    public Command Reverse() {
        return LoggedCommands.startEnd("Reverse", this::reverse, this::stop, this);
    }

    public Command RunFeederOnly() {
        return LoggedCommands.runEnd("Feeder Only",
            () -> { feederMotor.setControl(new VoltageOut(12.0).withEnableFOC(true)); },
            () -> { feederMotor.stopMotor(); });
    }

    public Command RunFloorOnly() {
        return LoggedCommands.runEnd("Floor Only",
            () -> { floorMotor.setControl(new VoltageOut(12.0).withEnableFOC(true)); },
            () -> { floorMotor.stopMotor(); });
    }
}