package frc.robot.superstructure;

import static frc.robot.Options.optHoldX;
import static frc.robot.Options.optWaitForAim;

import java.util.Set;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.Aiming;
import frc.robot.Constants;
import frc.robot.Field.Zone;
import frc.robot.Shift;
import frc.robot.subsystems.controls.Controls;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterMode;
import frc.robot.subsystems.swerve.Swerve;

public class Superstructure extends SubsystemBase {
    public static final Superstructure instance = new Superstructure();
    private Shooter shooter = Shooter.getInstance();
    private Intake intake = Intake.getInstance();
    private Feeder feeder = Feeder.getInstance();
    private final SendableChooser<ShotConfig> shotSelector = new SendableChooser<>();
    private boolean shiftWarned = false;
    private Shift currentShift = Shift.DISABLED;
    
    public static enum SuperState {
        DEFAULT,
        NONE
    }

    private SuperState superState = SuperState.DEFAULT;

    public enum ShotConfig {
		AUTO(0.0),
		CLOSE(1.55),
		TOWER(3.15),
		TRENCH(3.64);

		public final double distance;

		private ShotConfig(double distance) {
			this.distance = distance;
		}
	}

    ShotConfig activeShot = ShotConfig.AUTO;

    public Superstructure() {
        shotSelector.setDefaultOption("-- Auto --", ShotConfig.AUTO);
        for (ShotConfig config : ShotConfig.values()) {
            if (config != ShotConfig.AUTO) {
                shotSelector.addOption(config.toString(), config);
            }
        }
        SmartDashboard.putData("Shot Select", shotSelector);
    }

    // private Command SetSuperState(SuperState state) {
    //     return LoggedCommands.runOnce("Set Superstate to " + state, () -> superState = state);
    // }

    // private void setSuperStateDone(boolean interrupted) {
    //     if (interrupted) {
    //         DogLog.log(LoggedCommands.key, "Interrupted, setting superstate to NONE");
    //     } else {
    //         DogLog.log(LoggedCommands.key, "Completed, setting superstate to NONE");
    //     }
    //     superState = SuperState.NONE;
    // }

    public boolean hopperFull() {
        return shooter.getShotTimer() < (Constants.hopperShootingTime / 2.0);
    }

    public Command Intake() {
        return Commands.parallel(
            shooter.ResetCounter(),
            intake.RunIntake());
    }

    public Command DeliverToHub() {
        // TODO What happens if we lose alignment?
        return LoggedCommands.parallel("Deliver To Hub",
            shooter.Shoot(),
            intake.Pulse(),
            LoggedCommands.sequence("Wait then Feed",
                LoggedCommands.waitUntil("Wait for shooter ready", () -> shooter.getMode() == ShooterMode.SHOOTING),
                LoggedCommands.waitUntil("Wait for alignment", () -> !optWaitForAim.get() || Aiming.isHubAligned()),
                shooter.AdjustForLoad(),
                Commands.parallel(
                    Commands.either(
                        Swerve.instance.HoldX().asProxy(),
                        Commands.idle(),
                        optHoldX),
                    feeder.Feed())
            )
        );
    } 

    public Command Pass() {
        return LoggedCommands.parallel("Pass",
            shooter.Pass(),
            intake.Pulse(),
            LoggedCommands.sequence("Wait then Feed",
                LoggedCommands.waitUntil("Wait for shooter ready", () -> shooter.getMode() == ShooterMode.PASSING),
                LoggedCommands.waitUntil("Wait for alignment", () -> !optWaitForAim.get() || Aiming.isPassAligned()),
                shooter.AdjustForLoad(),
                feeder.Feed()
            )
        );
    }

    public Command FixedDelivery() {
        return LoggedCommands.parallel("Fixed Delivery To Hub",
            LoggedCommands.defer("Deferred Fixed Shot", () -> shooter.FixedShot(activeShot.distance), Set.of(shooter)),
            intake.Pulse(),
            LoggedCommands.sequence("Wait then Feed",
                LoggedCommands.waitUntil("Wait for shooter ready", () -> shooter.getMode() == ShooterMode.SHOOTING),
                shooter.AdjustForLoad(),
                feeder.Feed()
            )
        );       
    }

    public Command SmartShoot() {
        return LoggedCommands.either("Pass or Shoot",
            Pass(),
            Commands.either(
                DeliverToHub(),
                FixedDelivery(),
                () -> activeShot == ShotConfig.AUTO),
            () -> Pose.instance.getZone() == Zone.REMOTE);
    }

    public Command Unjam() {
        return LoggedCommands.parallel("Unjam",
            shooter.Reverse(),
            intake.Reverse(),
            feeder.Reverse());
    }

    @Override
    public void periodic() {
        DogLog.log("Superstructure/Super State", superState);
        activeShot = shotSelector.getSelected();
        DogLog.log("Superstructure/Active Shot", activeShot.toString());
        DogLog.log("Superstructure/Hopper Full", hopperFull());

        final double shiftWarning = 7.0;

        double matchTime = DriverStation.getMatchTime();
        Shift lastShift = currentShift;
        currentShift = Shift.lookup(matchTime);

        if (currentShift != lastShift) {
            shiftWarned = false;
        }

        double shiftTime = currentShift.timeLeft(matchTime);
        if (matchTime >= 0 && !shiftWarned && shiftTime < shiftWarning && currentShift.isActive() != currentShift.isNextActive()) {
            shiftWarned = true;
            CommandScheduler.getInstance().schedule(Controls.instance.Rumble());
            LED.triggerShiftWarning();
        }
    }
}