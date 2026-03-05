package frc.lib.util;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LynkMotor.MotorConfig;
import frc.robot.Ports;

public abstract class LynkSubsystem<T> extends SubsystemBase {
    private static final Map<String, LynkMotor> allMotors = new HashMap<String, LynkMotor>();
    protected String name;
    private final Map<String, LynkMotor> motors = new HashMap<String, LynkMotor>();

    public LynkSubsystem() {
        this.name = getClass().getSimpleName();
    }

    public LynkSubsystem(String name) {
        this.name = name;
    }

    public LynkMotor addMotor(String name, Ports port) {
        assert(!motors.containsKey(name));
        
        LynkMotor motor = new LynkMotor(port.id, port.bus);
        motors.put(name, motor);
        allMotors.put(name, motor);
        return motor;
    }

    public LynkMotor addMotor(String name, Ports port, TalonFXConfiguration config) {
        LynkMotor motor = addMotor(name, port);
        motor.getConfigurator().apply(config);
        return motor;
    }

    public LynkMotor addMotor(MotorConfig config) {
        assert(!motors.containsKey(config.name));

        LynkMotor motor = new LynkMotor(config);
        motors.put(config.name, motor);
        return motor;
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();

        DogLog.log(name + "/Current Command", currentCommand == null ? "None" : currentCommand.getName());

        motors.forEach((motorName, motor) -> {
            String logPrefix = name + "/" + motorName + " Motor/";
            DogLog.log(logPrefix + "Torque Current (Amps)", motor.getTorqueCurrent().getValue().in(Units.Amps));
            DogLog.log(logPrefix + "Stator Current (Amps)", motor.getStatorCurrent().getValue().in(Units.Amps));
            DogLog.log(logPrefix + "Supply Current (Amps)", motor.getSupplyCurrent().getValue().in(Units.Amps));
            DogLog.log(logPrefix + "Velocity (RPS)", motor.getVelocity().getValue().in(Units.RotationsPerSecond));
            DogLog.log(logPrefix + "Rotor Velocity (RPS)", motor.getRotorVelocity().getValue().in(Units.RotationsPerSecond));
            DogLog.log(logPrefix + "Voltage", motor.getMotorVoltage().getValue().in(Units.Volts));
            DogLog.log(logPrefix + "Position (rotations)", motor.getPosition().getValue().in(Units.Rotations));
            DogLog.log(logPrefix + "Position (degrees)", motor.getPosition().getValue().in(Units.Degrees));
            DogLog.log(logPrefix + "Temperature (C)", motor.getDeviceTemp().getValue().in(Units.Celsius));
            DogLog.log(logPrefix + "Stalled?", motor.checkStall());
        });
    }

    public static void globalPeriodic() {
        double statorCurrent = allMotors.values().stream().mapToDouble((motor) -> motor.getStatorCurrent().getValue().abs(Units.Amps)).sum();

        DogLog.log("Global/Stator Current (Amps)", statorCurrent);
    }
}