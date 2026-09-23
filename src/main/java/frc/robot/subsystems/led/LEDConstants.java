package frc.robot.subsystems.led;

import frc.lib.util.CustomFireAnimation;
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
    /* Fire animation parameters (shared between red and blue palettes for a consistent look) */
    public static final double fireFrameRate = 60.0;
    public static final double fireSparking = 1.0;
    public static final double fireCooling = 0.73;
    /* Animations */
    public static final CustomFireAnimation readyAnimation = new CustomFireAnimation(startIdx, totalLEDs - 1)
        .withBrightness(brightness)
        .withFrameRate(fireFrameRate)
        .withSparking(fireSparking)
        .withCooling(fireCooling);
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
