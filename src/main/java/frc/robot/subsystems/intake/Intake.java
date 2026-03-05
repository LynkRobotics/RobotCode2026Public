package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedCommands;
import frc.lib.util.LynkMotor;
import frc.lib.util.LynkSubsystem;
import frc.robot.Ports;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;

// TODO Default command to stop?

public class Intake extends LynkSubsystem<Intake> {
    private static Intake instance;
    private final LynkMotor motor, follower;
    private final VoltageOut intakeControl = new VoltageOut(0).withEnableFOC(true);
    private IntakeMode mode = IntakeMode.IDLE;
    private final Timer pulseTimer = new Timer();
    private boolean pulseIn = true;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public Intake() {
        super("Intake");

        assert(instance == null) : "Intake instance already exists";
        instance = this;

        motor = addMotor(IntakeConstants.getMotorConfig());
        follower = addMotor("Intake B", Ports.INTAKE_B);

        motor.stopMotor();
        follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void run() {
        motor.setControl(intakeControl.withOutput(IntakeConstants.intakeVoltage).withEnableFOC(true));
        mode = IntakeMode.ACTIVE;
    }

    public void stop() {
        motor.stopMotor();
        mode = IntakeMode.IDLE;
        pulseTimer.stop();
    }

    public IntakeMode getMode() {
        return mode;
    }

    public Command RunIntake() {
        return LoggedCommands.runEnd("Run Intake", this::run, this::stop, this);
    }

    private void startPulse(boolean in) {
        pulseIn = in;
        pulseTimer.restart();
        motor.setControl(intakeControl.withOutput(pulseIn ? IntakeConstants.intakeVoltage : IntakeConstants.intakeReverseVoltage));
        mode = IntakeMode.PULSING;
    }

    public Command Pulse() {
        return LoggedCommands.startEnd("Pulse Intake", () -> startPulse(true), this::stop, this);
    }

    private void reverse() {
        motor.setControl(intakeControl.withOutput(IntakeConstants.intakeReverseVoltage));
        mode = IntakeMode.REVERSE;
    }

    public Command Reverse() {
        return LoggedCommands.startEnd("Reverse Intake", this::reverse, this::stop, this);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (mode == IntakeMode.PULSING) {
            if (pulseTimer.hasElapsed(pulseIn ? IntakeConstants.pulseInTime : IntakeConstants.pulseOutTime)) {
                startPulse(!pulseIn);
            }
        }

        DogLog.log(name + "/Mode", mode.toString());
        DogLog.log(name + "/Pulse In", pulseIn);
        DogLog.log(name + "/Pulse Timer", pulseTimer.get());
    }
}