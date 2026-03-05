package frc.lib.util;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import dev.doglog.DogLog;
import frc.robot.subsystems.led.LEDConstants;

public class CANdleGroup {
    private CANdle[] candles;
    private final ControlRequest clearAnimation = new EmptyAnimation(0);
    
    public CANdleGroup(CANdle... candles) {
        this.candles = candles;
        applyConfigs();
    }

    private void applyConfigs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.BrightnessScalar = LEDConstants.brightness;
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
        config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;

        for (CANdle candle : candles) {
            candle.getConfigurator().apply(config);
        }
    }

    public void animate(ControlRequest animation) {
        DogLog.log("LED/Status", "Setting animation");
        for (CANdle candle : candles) {
            candle.setControl(animation);
        }
    }

    public void clearAnimation() {
        DogLog.log("LED/Status", "Clearing animation");
        for (CANdle candle : candles) {
            candle.setControl(clearAnimation);
        }
    }

    public void setLEDs(RGBWColor color, int startIdx, int count) {
        DogLog.log("LED/Status", "Set " + count + " LEDs at " + startIdx + " to (" + color.Red + "," + color.Green + "," + color.Blue + ")");
        for (CANdle candle : candles) {
            candle.setControl(new SolidColor(startIdx, startIdx + count - 1).withColor(color));
        }
    }

    public void setLEDs(int r, int g, int b, int w, int startIdx, int count) {
        setLEDs(new RGBWColor(r, g, b, w), startIdx, count);
    }
}