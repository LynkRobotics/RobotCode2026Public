package frc.lib.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Ports;

public class LynkMotor extends TalonFX {
    private String name = null;
    private StallConfig stallConfig = null;
    private Debouncer stallDebouncer = null;
    private boolean zeroing = false;

    public static class MotorConfig {
        public String name = null;
        public Ports port = null;
        public TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        public StallConfig stallConfig = new StallConfig();
    }

    public static class StallConfig {
        public Current stallCurrent = null; // Current which, when exceeded, will be considered a stall
        public AngularVelocity velocityEpsilon = null; // Velocity which, when not reached, will be considered a stall
        public Time stallTime = Units.Seconds.of(0); // Amount of time conditions must be true to be considered a stall
        public ControlRequest zeroControl = null; // How to control the motor to find the "zero" position
        public Angle zeroPosition = Angle.ofRelativeUnits(0, Units.Rotations); // What the motor's position when "zeroed" should be
    }
    
    public LynkMotor(int id, CANBus bus) {
        super(id, bus);
        name = bus.getName() + "-" + id;
    }

    public LynkMotor(MotorConfig config) {
        super(config.port.id, config.port.bus);
        setConfig(config);
    }

    public void setConfig(MotorConfig config) {
        if (name == null) {
            name = config.name != null ? config.name : config.port.name();
        }
        getConfigurator().apply(config.talonConfig);
        setStallConfig(config.stallConfig);
    }

    public void setStallConfig(StallConfig config) {
        this.stallConfig = config;
        stallDebouncer = new Debouncer(config.stallTime.in(Units.Seconds), DebounceType.kRising);
    }

    public void resetStall() {
        if (stallDebouncer != null) stallDebouncer.calculate(false);
    }

    public boolean stallConditions() {
        boolean stalled = false;

        if (stallConfig != null) {
            if (stallConfig.stallCurrent != null) {
                if (stallConfig.stallCurrent.abs(Units.Amps) > 0) {
                    stalled = stalled || getStatorCurrent().getValue().gte(stallConfig.stallCurrent);
                } else {
                    stalled = stalled || getStatorCurrent().getValue().lte(stallConfig.stallCurrent);
                }
            }
            if (stallConfig.velocityEpsilon != null && getMotorVoltage().getValueAsDouble() != 0) {
                stalled = stalled || (Math.abs(getVelocity().getValue().in(Units.RotationsPerSecond)) < stallConfig.velocityEpsilon.in(Units.RotationsPerSecond));
            }
        }

        return stalled;
    }

    public boolean checkStall() {
        return stallDebouncer == null ? false : stallDebouncer.calculate(stallConditions());
    }

    public Command ZeroAndWait() {
        return Commands.either(
            LoggedCommands.sequence("Zero motor " + name,
                Commands.runOnce(this::startZero),
                Commands.waitUntil(this::checkStall)),
            LoggedCommands.log("Motor " + name + " not configured for zeroing"),
            () -> stallConfig != null && stallConfig.zeroControl != null)
            .handleInterrupt(() -> {
                zeroing = false;
                stopMotor();
            });
    }

    public void startZero() {
        if (stallConfig != null && stallConfig.zeroControl != null) {
            setControl(stallConfig.zeroControl);
            zeroing = true;
        }
    }

    public boolean isZeroing() {
        return zeroing;
    }

    public boolean checkZero() {
        if (zeroing && checkStall()) {
            zeroing = false;
            stopMotor();
            return true;
        }

        return false;
    }
}