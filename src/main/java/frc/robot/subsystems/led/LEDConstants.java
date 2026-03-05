package frc.robot.subsystems.led;

import frc.robot.Constants;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

public class LEDConstants {
    /* LED arrangement */
    public static final int startIdx = 8;
    public static final int numLEDs = 54;
    public static final int totalLEDs = startIdx + numLEDs;
    public static final double brightness = Constants.atHQ ? 0.60 : 1.00;
    /* Animations */
    public static final ControlRequest readyAnimation = new FireAnimation(startIdx, totalLEDs - 1)
        .withBrightness(brightness).withFrameRate(45).withSparking(0.8).withCooling(0.28);
    public static final ControlRequest serviceModeAnimation = new ColorFlowAnimation(startIdx, totalLEDs - 1)
        .withColor(new RGBWColor(0, 25, 25, 0)).withFrameRate(100);
    public static final ControlRequest intakeAnimation = new LarsonAnimation(startIdx, totalLEDs - 1)
        .withColor(new RGBWColor(255, 64, 0, 0)).withFrameRate(125).withBounceMode(LarsonBounceValue.Front).withSize(15);
    /* Misc */
    public static final double blinkRate = 0.2; // Regular blink rate
    public static final double errorBlinkRate = 0.1; // Blink rate for errors and warnings
    public static final double tempStateTime = 0.70; // How long for warnings and errors
    public static final double shiftCountdown = 3.0;
}
