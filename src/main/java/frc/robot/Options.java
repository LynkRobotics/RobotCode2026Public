package frc.robot;

import frc.lib.util.TunableOption;

public final class Options {
    /* Global Toggleable Options and their defaults */
    public static final TunableOption optServiceMode = new TunableOption("Service Mode", false);
    public static final TunableOption optUseTrigVision = new TunableOption("Use Trig Vision", false);
    public static final TunableOption optMirrorAuto = new TunableOption("Mirror Auto to Left", false);
    public static final TunableOption optAutoAiming = new TunableOption("Automatically Aim", true);
    public static final TunableOption optWaitForAim = new TunableOption("Wait for Aim", true);
    public static final TunableOption optPacManMode = new TunableOption("Pac-Man Mode", false);
    public static final TunableOption optHubActive = new TunableOption("Hub Active (Testing)", true);
    public static final TunableOption optLimitRotation = new TunableOption("Limit Rotation Speed", true);
    public static final TunableOption optHoldX = new TunableOption("Hold X on Shoot", false);
    public static final TunableOption optHoldAngle = new TunableOption("Hold Angle", true);
    public static final TunableOption optTestAiming = new TunableOption("Test Aiming", false);
}