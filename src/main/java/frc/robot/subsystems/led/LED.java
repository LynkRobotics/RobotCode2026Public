// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import static frc.robot.Options.optServiceMode;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Shift;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterMode;
import frc.lib.util.CANdleGroup;

/**
 * The LED subsystem controls the LED lights on the robot. It manages various LED states,
 * animations, and colors to visually indicate the robot's status, warnings, and other
 * conditions during operation.
 * 
 * <p>This subsystem uses a CANdleGroup to control the LEDs and supports features such as:
 * <ul>
 *   <li>Setting static colors</li>
 *   <li>Animating LEDs</li>
 *   <li>Blinking LEDs for warnings, errors, and information</li>
 *   <li>Dynamic state transitions based on robot conditions</li>
 * </ul>
 * 
 * <p>Key features:
 * <ul>
 *   <li>Predefined LED states (e.g., STARTUP, DISABLED, NORMAL, ENDGAME, etc.)</li>
 *   <li>Support for temporary states with timers</li>
 *   <li>Integration with SmartDashboard for manual control and debugging</li>
 *   <li>Automatic handling of endgame notifications and service modes</li>
 * </ul>
 * 
 * <p>Usage:
 * <ul>
 *   <li>Call {@link #triggerError()}, {@link #triggerWarning()}, or {@link #triggerInfo()} to set temporary states.</li>
 *   <li>Use {@link #setColor(Color)} to manually set LED colors.</li>
 *   <li>The {@link #periodic()} method automatically updates the LEDs based on the current state.</li>
 * </ul>
 * 
 * <p>Dependencies:
 * <ul>
 *   <li>Requires the CANdleGroup class for LED control.</li>
 *   <li>Relies on constants defined in {@code LEDConstants} and {@code Constants}.</li>
 * </ul>
 * 
 * <p>Note: This subsystem assumes the presence of a DriverStation and other robot-specific
 * components for determining the robot's operational state.
 */
public class LED extends SubsystemBase {
    public static final LED instance = new LED();
    
    /** Creates a new LEDSubsystem. */
    private static CANdleGroup leds;
    private static final Timer tempStateTimer = new Timer();
    private static final Timer blinkTimer = new Timer();

    private static LEDState state = LEDState.STARTUP;
    private static LEDState lastState = null;
    private static boolean blinkOff = false;

    private SendableChooser<Color> colorChooser = new SendableChooser<Color>();

    /**
     * Represents a set of predefined colors with their corresponding RGB values.
     * Each color is defined by its red (r), green (g), and blue (b) components.
     */
    private enum Color {
        off(0, 0, 0),
        red(255, 0, 0),
        green(0, 255, 0),
        citrus(0x43, 0xAF, 0x1C),
        hightideTeal(44, 162, 165),
        blue(0, 0, 255),
        cheesyBlue(0, 112, 255),
        cyan(0, 255, 255),
        magenta(255, 0, 255),
        yellow(255, 225, 0),
        dimYellow(150, 150, 0),
        white(250, 250, 250),
        brightWhite(255, 255, 255),
        dim(10, 10, 10),
        lynk(255, 64, 0),
        service(0, 10, 10),
        disabled(200, 0, 0);

        public final RGBWColor rgbw;

        Color(int r, int g, int b) {
            this.rgbw = new RGBWColor(r, g, b, 255);
        }
    }

    private static class LEDConfig {
        ControlRequest animation = null;
        Color color = null;
        Color bgColor = null;
        boolean blink = false;
        double pct = 1.0;

        LEDConfig(Color color) {
            this.color = color;
        }

        LEDConfig(Color color, boolean blink) {
            this.color = color;
            this.blink = true;
        }

        LEDConfig(Color color, Color bgColor) {
            this.color = color;
            this.bgColor = bgColor;
        }

        LEDConfig(Color color, Color bgColor, boolean blink) {
            this.color = color;
            this.bgColor = bgColor;
            this.blink = true;
        }

        @SuppressWarnings("unused")
        LEDConfig(Color color, double pct) {
            this.color = color;
            this.pct = pct;
        }

        LEDConfig(ControlRequest animation) {
            this.animation = animation;
        }
    }

    public enum LEDState {
        STARTUP(new LEDConfig(LEDConstants.readyAnimation)),
        DISABLED(new LEDConfig(Color.disabled)),
        ACTIVE_SHOOTING(new LEDConfig(Color.yellow, Color.magenta, true)),
        INACTIVE_SHOOTING(new LEDConfig(Color.magenta, Color.yellow, true)),
        INTAKING(new LEDConfig(LEDConstants.intakeAnimation)),
        ACTIVE_IDLE(new LEDConfig(Color.yellow, Color.magenta)),
        INACTIVE_IDLE(new LEDConfig(Color.magenta, Color.yellow)),
        SERVICE_MODE(new LEDConfig(LEDConstants.serviceModeAnimation)),
        SERVICE_DISABLED(new LEDConfig(Color.service)),
        ERROR(new LEDConfig(Color.red, true)),
        WARNING(new LEDConfig(Color.yellow, true)),
        INFO(new LEDConfig(Color.white, true)),
        SHIFT_WARNING(new LEDConfig(Color.cheesyBlue, true));

        public final LEDConfig config;

        LEDState(LEDConfig config) {
            this.config = config;
        } 
    }

    public LED() {
        // Control both CANdles in the same way
        leds = new CANdleGroup(
                new CANdle(Ports.CANDLE.id, Ports.CANDLE.bus));

        // Set color on the CANdles themselves
        setColor(Color.lynk, 0, LEDConstants.startIdx);

        if (Constants.fullDashboard) {
            colorChooser = new SendableChooser<Color>();
            SmartDashboard.putNumber("LED/start", 0);
            SmartDashboard.putNumber("LED/count", 100);
            for (Color color : Color.values()) {
                colorChooser.addOption(color.name(), color);
            }
            SmartDashboard.putData("LED/color", colorChooser);
            SmartDashboard.putData("LED/Set LED color",
                LoggedCommands.runOnce("Set LED color",
                () -> {
                    int start = (int)SmartDashboard.getNumber("LED/start", 0);
                    int cnt = (int)SmartDashboard.getNumber("LED/count", 100);
                    Color color = colorChooser.getSelected();
                    setColor(color, start, cnt);
                }).ignoringDisable(true));
        }

        // Set color on the LEDs
        setColor(Color.lynk, LEDConstants.startIdx, LEDConstants.numLEDs);

        // SmartDashboard.putString("LED Mode Override", "");
        // SmartDashboard.putNumber("LED Pct Override", -1.0);
    }

    private static void setColor(Color color, int startIdx, int count) {
        if (count > 0) {
            leds.setLEDs(color.rgbw, startIdx, count);
        }
    }

    private static void setColor(Color color, int count) {
        setColor(color, LEDConstants.startIdx, count);
    }

    private static void setColor(Color color) {
        setColor(color, LEDConstants.numLEDs);
    }

    private static void setColor(Color color, double pct) {
        setColors(color, pct, Color.off);
    }

    private static void setColors(Color color, double pct, Color bgColor) {
        int ledCount = (int)Math.round(LEDConstants.numLEDs * pct);

        setColor(bgColor, LEDConstants.startIdx + ledCount, LEDConstants.numLEDs - ledCount);
        setColor(color, ledCount);
    }

    public static void triggerError() {
        state = LEDState.ERROR;
        tempStateTimer.restart();
    }

    public static void triggerWarning() {
        state = LEDState.WARNING;
        tempStateTimer.restart();
    }

    public static void triggerInfo() {
        state = LEDState.INFO;
        tempStateTimer.restart();
    }

    public static void triggerShiftWarning() {
        state = LEDState.SHIFT_WARNING;
        tempStateTimer.restart();
    }

    @Override
    public void periodic() {
        double pctLeft = 1.0;
        boolean disabledNext = false;

        // Determine the proper LED state
        if (optServiceMode.get()) {
            state = DriverStation.isDisabled() ? LEDState.SERVICE_DISABLED : LEDState.SERVICE_MODE;
        } else if (DriverStation.isDisabled()) {
            if (state != LEDState.STARTUP && state != LEDState.DISABLED) {
                DogLog.log("LED/Status", "Transitioned to DISABLED from " + state);
                state = LEDState.DISABLED;
            }
        } else {
            // Check if the temporary state should be cleared
            if (tempStateTimer.isRunning() && tempStateTimer.hasElapsed(LEDConstants.tempStateTime)) {
                tempStateTimer.stop();
            }
            // Compute the proper state if's not temporarily overridden
            if (!tempStateTimer.isRunning()) {
                double timeLeft = DriverStation.getMatchTime();
                Shift shift = Shift.lookup(timeLeft);
                boolean hubActive = shift.isActive();
                double shiftLeft = shift.timeLeft(timeLeft);
                boolean nextActive = shift.isNextActive();
                
                disabledNext = shift.disabledNext();
                if (shiftLeft <= LEDConstants.shiftCountdown && hubActive != nextActive && shiftLeft >= 0) {
                    pctLeft = shiftLeft / LEDConstants.shiftCountdown;
                }

                if (pctLeft == 1.0 && Intake.getInstance().getMode() == IntakeMode.ACTIVE) {
                    state = LEDState.INTAKING;
                } else {
                    ShooterMode mode = Shooter.getInstance().getMode();
                    
                    if (mode == ShooterMode.SHOOTING || mode == ShooterMode.PASSING) {
                        state = hubActive ? LEDState.ACTIVE_SHOOTING : LEDState.INACTIVE_SHOOTING;
                    } else {
                        state = hubActive ? LEDState.ACTIVE_IDLE : LEDState.INACTIVE_IDLE;
                    }
                }
            }
        }

        // String override = SmartDashboard.getString("LED Mode Override", "");
        // if (override != null && override.length() > 0) {
        //     state = LEDState.valueOf(override);
        // }
        // double overridePct = SmartDashboard.getNumber("LED Pct Override", -1.0);
        // if (overridePct >= 0.0) {
        //     pctLeft = overridePct;
        // }

        // If changing to a blinking state, restart the blink
        boolean blinkToggle = false;
        if (state != lastState) {
            if (state.config.blink) {
                blinkOff = false;
                blinkTimer.restart();
            } else {
                blinkTimer.stop();
            }
        } else {
            // Blink the LEDs if the blink interval has elapsed
            if (blinkTimer.isRunning() && blinkTimer.advanceIfElapsed((state == LEDState.ERROR || state == LEDState.WARNING || state == LEDState.INFO || state == LEDState.SHIFT_WARNING) ? LEDConstants.errorBlinkRate : LEDConstants.blinkRate)) {
                blinkOff = !blinkOff;
                blinkToggle = true;
            }
        }

        // Change the LEDs if the state has changed
        if (state != lastState || pctLeft != 1.0 || blinkToggle) {
            if (lastState == null || lastState.config.animation != null) {
                leds.clearAnimation();
            }
            if (state.config.animation != null) {
                leds.animate(state.config.animation);
            } else {
                if (state.config.blink && blinkOff) {
                    if (blinkToggle) {
                        setColor(Color.off);
                    }
                } else if (state.config.pct == 1.0) {
                    if (pctLeft == 1.0) {
                        setColor(state.config.color);
                    } else if (!state.config.blink || !blinkOff) {
                        setColors(state.config.color, pctLeft, disabledNext ? Color.disabled : state.config.bgColor);
                    }
                } else {
                    setColor(state.config.color, state.config.pct);
                }
            }

        }

        DogLog.log("LED/State", state.toString());
        lastState = state;
    }    
}