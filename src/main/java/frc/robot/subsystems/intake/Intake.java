package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.lib.util.LynkMotor;
import frc.lib.util.LynkSubsystem;
import frc.robot.Ports;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.swerve.Swerve;

// TODO Default command to stop?

public class Intake extends LynkSubsystem<Intake> {
    private static Intake instance;
    private final LynkMotor motor, follower;
    private final VoltageOut intakeControl = new VoltageOut(0).withEnableFOC(true);
    private IntakeMode mode = IntakeMode.IDLE;
    private final Timer pulseTimer = new Timer();
    private boolean pulseIn = true;
    private final Timer currentDiffTimer = new Timer();

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

    private Command IntakeUnjam() {
        return LoggedCommands.deadline("Unjam Intake",
            Commands.waitSeconds(IntakeConstants.intakeCurrentFixTime),
            Commands.runOnce(() -> LoggedAlert.Warning("Intake", "Intake Jam", "Intake jam detected; reversing")),
            Reverse(),
            Swerve.instance.Stop()
        );
    }

    @Override
    public void periodic() {
        super.periodic();

        if (mode == IntakeMode.PULSING) {
            if (pulseTimer.hasElapsed(pulseIn ? IntakeConstants.pulseInTime : IntakeConstants.pulseOutTime)) {
                startPulse(!pulseIn);
            }
        }

        double currentDiff = Math.abs(motor.getStatorCurrent().getValueAsDouble() - motor.getSupplyCurrent().getValueAsDouble());
        if (currentDiff > IntakeConstants.intakeCurrentDiffWarningLimit) {
            if (!currentDiffTimer.isRunning()) {
                currentDiffTimer.start();
            } else if (currentDiffTimer.hasElapsed(IntakeConstants.intakeCurrentDiffWarningTime)) {
                DogLog.log("Intake Current Diff Warning", Timer.getTimestamp());
                currentDiffTimer.restart();
                if (mode == IntakeMode.ACTIVE) {
                    LoggedAlert.Error("Intake", "Current Warning", "Excessive Intake Current Detected");
                    if (DriverStation.isTeleopEnabled()) {
                        CommandScheduler.getInstance().schedule(IntakeUnjam());
                    }
                }
            }
            SmartDashboard.putBoolean("Intake Current Warning", true);
        } else {
            SmartDashboard.putBoolean("Intake Current Warning", false);
            if (currentDiffTimer.isRunning()) {
                currentDiffTimer.stop();
                currentDiffTimer.reset();
            }
        }

        DogLog.log(name + "/Mode", mode.toString());
        DogLog.log(name + "/Pulse In", pulseIn);
        DogLog.log(name + "/Pulse Timer", pulseTimer.get());
        DogLog.log(name + "/Current Diff", currentDiff);
        DogLog.log(name + "/Current Diff Timer", currentDiffTimer.get());
    }
}