package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.LoggedCommands;
import frc.lib.util.LynkMotor;
import frc.lib.util.LynkSubsystem;
import frc.robot.Ports;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterMode;

public class Shooter extends LynkSubsystem<Shooter> {
    private static Shooter instance;
    private final LynkMotor flywheelMotorA;
    private final LynkMotor flywheelMotorB, flywheelMotorC, flywheelMotorD;
    // private final LynkMotor kickerMotor;

    private boolean loaded = false;
    private ShooterMode shooterMode = ShooterMode.STOPPED;
    private ShooterMode pendingMode = null;

    private final VelocityVoltage flywheelVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private final MotionMagicVelocityVoltage flywheelVelocitySmooth = new MotionMagicVelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut flywheelVoltage = new VoltageOut(0).withEnableFOC(true);
    // private final PositionVoltage kickerPosition = new PositionVoltage(0).withEnableFOC(true);
    // TODO Use MotionMagicExpoVoltage for kicker

    // SysIdRoutine for flywheel characterization
    private SysIdRoutine sysIdRoutine;

    private double targetRPM = 0.0;
    private Timer spinupTimer = new Timer();
    private int spinupTimeoutCount = 0;
    private Timer shotTimer = new Timer();

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public ShooterMode getMode() {
        return shooterMode;
    }
    
    public Shooter() {
        super("Shooter");

        assert(instance == null) : "Shooter instance already exists";
        instance = this;

        flywheelMotorA = addMotor(ShooterConstants.getFlywheelMotorConfig());
        flywheelMotorB = addMotor("Flywheel B", Ports.FLYWHEEL_B);
        flywheelMotorC = addMotor("Flywheel C", Ports.FLYWHEEL_C);
        flywheelMotorD = addMotor("Flywheel D", Ports.FLYWHEEL_D);
        // kickerMotor = addMotor(ShooterConstants.getKickerMotorConfig());

        flywheelMotorA.stopMotor(); // TODO Default to idle speed
        flywheelMotorB.setControl(new Follower(flywheelMotorA.getDeviceID(), MotorAlignmentValue.Aligned));
        flywheelMotorC.setControl(new Follower(flywheelMotorA.getDeviceID(), MotorAlignmentValue.Opposed));
        flywheelMotorD.setControl(new Follower(flywheelMotorA.getDeviceID(), MotorAlignmentValue.Opposed));

        // kickerMotor.stopMotor(); // TODO Stall to find zero

        SmartDashboard.putData("Stop", LoggedCommands.runOnce("Stop", this::stop));
        SmartDashboard.putNumber("Direct Voltage", 0);
        SmartDashboard.putData("Set Voltage", LoggedCommands.runOnce("Set Voltage", () -> flywheelMotorA.setControl(flywheelVoltage.withOutput(SmartDashboard.getNumber("Direct Voltage", 0)))));
        SmartDashboard.putNumber("Direct RPM", 0);
        SmartDashboard.putData("Set RPM", LoggedCommands.runOnce("Set RPM", () -> setTargetRPM(SmartDashboard.getNumber("Direct RPM", 0))));
    
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                null,     // Use default step voltage (7 V)
                null,         // Use default timeout (10 s)
                (state) -> DogLog.log(name + "/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> flywheelMotorA.setControl(flywheelVoltage.withOutput(volts)),
                null,        // No loggable measurements for flywheel
                this
            )
        );

        // Add SysIdRoutine commands to SmartDashboard
        SmartDashboard.putData("Shooter SysId Quasistatic Forward", sysIdQuasistaticForward());
        SmartDashboard.putData("Shooter SysId Quasistatic Reverse", sysIdQuasistaticReverse());
        SmartDashboard.putData("Shooter SysId Dynamic Forward", sysIdDynamicForward());
        SmartDashboard.putData("Shooter SysId Dynamic Reverse", sysIdDynamicReverse());

        setDefaultCommand(Idle());
    }

    public void runIdle() {
        setTargetRPM(ShooterConstants.idleRPM, true);
        // kickerMotor.set(0);

        shooterMode = ShooterMode.IDLE;
        pendingMode = null;
        loaded = false;
    }

    public void setTargetRPM(double rpm) {
        setTargetRPM(rpm, false);
    }

    public void setTargetRPM(double rpm, boolean smooth) {
        targetRPM = rpm;

        if (smooth) {
            flywheelMotorA.setControl(flywheelVelocitySmooth.withVelocity(targetRPM / 60.0));
        } else {
            flywheelMotorA.setControl(flywheelVelocity.withVelocity(targetRPM / 60.0).withSlot(loaded ? 1 : 0));
        }
    }

    public void pass() {
        shoot(Pose.instance.passDistance(), ShooterMode.PASSING);
    }

    public void shoot() {
        shoot(Pose.instance.hubDistance(), ShooterMode.SHOOTING);
    }

    public void shoot(double distance) {
        shoot(distance, ShooterMode.SHOOTING);
    }

    public void shoot(double distanceMeters, ShooterMode mode) {
        double shooterRPM = SmartDashboard.getNumber("Direct RPM", 0);

        if (shooterRPM <= 0) {
            ShooterConstants.ShooterSetpoint setpoint = (mode == ShooterMode.PASSING)
                ? ShooterConstants.passingTable.lookup(distanceMeters)
                : ShooterConstants.shootingTable.lookup(distanceMeters);
            if (setpoint == null) {
                stop();
                return;
            }
            shooterRPM = setpoint.flywheelRPM();
        }

        setTargetRPM(shooterRPM);

        // kickerPosition.Position = setpoint.kickerAngleDeg() / 360.0;
        // kickerMotor.setControl(kickerPosition);

        if (shooterMode != mode && shooterMode != ShooterMode.SPINNING) {
            shooterMode = ShooterMode.SPINNING;
            pendingMode = mode;
        }
    }

    public Command Shoot() {
        return LoggedCommands.startRun("Shoot", shotTimer::start, this::shoot, this).finallyDo(shotTimer::stop);
    }

    public Command FixedShot(double distance) {
        return LoggedCommands.startRun("Fixed Shot @ " + String.format("%1.1f", distance), shotTimer::start, () -> shoot(distance), this).finallyDo(shotTimer::stop);
    }

    public Command Pass() {
        return LoggedCommands.startRun("Pass", shotTimer::start, this::pass, this).finallyDo(shotTimer::stop);
    }

    public Command Idle() {
        return LoggedCommands.startEnd("Idle shooter", this::runIdle, () -> {}, this);
    }

    public Command ResetCounter() {
        return LoggedCommands.runOnce("Reset Shot Counter", shotTimer::reset);
    }

    public double getShotTimer() {
        return shotTimer.get();
    }

    public boolean isActive() {
        return shotTimer.isRunning();
    }

    /**
     * Configure follower motors for SysId testing
     * When enabled, follower motors will be stopped to prevent them from interfering with characterization
     */
    public void configureFollowersForSysId(boolean enableSysId) {
        if (enableSysId) {
            // Stop follower motors during SysId testing to get accurate characterization of main motor
            flywheelMotorB.stopMotor();
            flywheelMotorC.stopMotor();
            flywheelMotorD.stopMotor();
        } else {
            // Re-enable follower configuration after SysId testing
            flywheelMotorB.setControl(new Follower(flywheelMotorA.getDeviceID(), MotorAlignmentValue.Aligned));
            flywheelMotorC.setControl(new Follower(flywheelMotorA.getDeviceID(), MotorAlignmentValue.Opposed));
            flywheelMotorD.setControl(new Follower(flywheelMotorA.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    /**
     * SysIdRoutine command that properly configures followers before and after testing
     */
    public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .beforeStarting(() -> configureFollowersForSysId(true))
            .andThen(() -> configureFollowersForSysId(false));
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
            .beforeStarting(() -> configureFollowersForSysId(true))
            .andThen(() -> configureFollowersForSysId(false));
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
            .beforeStarting(() -> configureFollowersForSysId(true))
            .andThen(() -> configureFollowersForSysId(false));
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
            .beforeStarting(() -> configureFollowersForSysId(true))
            .andThen(() -> configureFollowersForSysId(false));
    }

    public void stop() {
        targetRPM = 0.0;
        flywheelMotorA.stopMotor();

        // kickerMotor.set(0); // Move to position 0

        shooterMode = ShooterMode.STOPPED;
    }

    public void reverse() {
        flywheelMotorA.setControl(flywheelVoltage.withOutput(ShooterConstants.reverseVoltage));
    }

    public Command Reverse() {
        return LoggedCommands.startEnd("Reverse shooter", this::reverse, () -> {}, this);
    }

    private boolean nearSpeed() {
        double currentRPM = flywheelMotorA.getVelocity().getValue().in(Units.RPM);
        return Math.abs(currentRPM - targetRPM) <= ShooterConstants.acceptableDeltaRPM;
    }

    private boolean atSpeed() {
        double currentRPM = flywheelMotorA.getVelocity().getValue().in(Units.RPM);
        return currentRPM >= targetRPM;
    }

    public void notifyLoaded() {
        loaded = true;
    }

    public Command AdjustForLoad() {
        return LoggedCommands.runOnce("Adjusting shooter for load", this::notifyLoaded);
    }

    public void periodic() {
        super.periodic();

        if (shooterMode == ShooterMode.SPINNING) {
            assert(pendingMode != null) : "Pending mode should not be null when spinning up";
            if (atSpeed()) {
                shooterMode = pendingMode;
                pendingMode = null;
                spinupTimer.stop();
            } else if (nearSpeed()) {
                // TODO Add timeout to prevent getting stuck in SPINNING mode
                if (spinupTimer.isRunning()) {
                    if (spinupTimer.hasElapsed(ShooterConstants.maxSpinUpTime)) {
                        spinupTimeoutCount++;
                        shooterMode = pendingMode;
                        pendingMode = null;
                        spinupTimer.stop();
                    }
                } else {
                    spinupTimer.reset();
                }
            }
        }

        DogLog.log(name + "/Target RPM", targetRPM);
        DogLog.log(name + "/Target RPS", targetRPM / 60.0);
        DogLog.log(name + "/Current RPM", flywheelMotorA.getVelocity().getValue().in(Units.RPM));
        DogLog.log(name + "/Mode", shooterMode);
        DogLog.log(name + "/Spinup Timeouts", spinupTimeoutCount);
        DogLog.log(name + "/Shot Timer", getShotTimer());
        SmartDashboard.putBoolean(name + "/Near Speed", nearSpeed());
        SmartDashboard.putBoolean(name + "/At Speed", atSpeed());
    }
}