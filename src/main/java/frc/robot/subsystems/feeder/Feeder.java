package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final Timer feederReverseTimer = new Timer();

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

        SmartDashboard.putData("Feeder/Deploy Intake", DeployIntake());
    }

    public void feed() {
        // floorMotor.set(FeederConstants.floorSpeed);
        // feederMotor.set(FeederConstants.feederSpeed);
        feederReverseTimer.stop();
        feederMotor.setControl(new VoltageOut(12.0).withEnableFOC(true));
        floorMotor.setControl(new VoltageOut(12.0).withEnableFOC(true));
    }

    public void stop() {
        feederReverseTimer.stop();
        floorMotor.stopMotor();
        feederMotor.stopMotor();
    }

    public Command Feed() {
        return LoggedCommands.startEnd("Feed", this::feed, () -> {
            feederAutoReverse();
            floorMotor.stopMotor();
        }, this);
    }

    private void reverse() {
        feederReverseTimer.stop();
        feederMotor.setControl(new VoltageOut(-6.0).withEnableFOC(true));
        floorMotor.setControl(new VoltageOut(-6.0).withEnableFOC(true));
    }

    public Command Reverse() {
        return LoggedCommands.startEnd("Reverse", this::reverse, this::stop, this);
    }

    private void feederAutoReverse() {
        feederReverseTimer.restart();
        feederMotor.setControl(new VoltageOut(-6.0).withEnableFOC(true));
    }

    public Command DeployIntake() {
        return LoggedCommands.startEnd("Deploy Intake",
            () -> floorMotor.setControl(new VoltageOut(FeederConstants.intakeDeployVoltage).withEnableFOC(true)),
            floorMotor::stopMotor,
            this).withTimeout(FeederConstants.intakeDeployTime);
    }

    public Command RunFeederOnly() {
        return LoggedCommands.runEnd("Feeder Only",
            () -> {
                feederReverseTimer.stop();
                feederMotor.setControl(new VoltageOut(12.0).withEnableFOC(true));
            },
            feederMotor::stopMotor);
    }

    public Command RunFloorOnly() {
        return LoggedCommands.runEnd("Floor Only",
            () -> {
                floorMotor.setControl(new VoltageOut(12.0).withEnableFOC(true));
            },
            floorMotor::stopMotor);
    }

    public void periodic() {
        super.periodic();

        DogLog.log(name + "/AutoReverse timer", feederReverseTimer.get());
        if (feederReverseTimer.isRunning() && feederReverseTimer.hasElapsed(FeederConstants.autoReverseTime)) {
            feederReverseTimer.stop();
            feederMotor.stopMotor();
        }
    }
}