package frc.robot.subsystems.swerve;

import frc.lib.util.LoggedCommands;
import frc.lib.util.SwerveModule;
import frc.robot.Constants;
import frc.robot.subsystems.pose.Pose;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.SignalLogger;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
    public static final Swerve instance = new Swerve();
    public ChassisSpeeds lastSpeeds = new ChassisSpeeds();
    
    public SwerveModule[] mSwerveMods;

    public Swerve() {
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0.constants),
            new SwerveModule(1, SwerveConstants.Mod1.constants),
            new SwerveModule(2, SwerveConstants.Mod2.constants),
            new SwerveModule(3, SwerveConstants.Mod3.constants)
        };
        // SmartDashboard.putData("Swerve Drive", new Sendable() {
        //     @Override
        //     public void initSendable(SendableBuilder builder) {
        //         builder.setSmartDashboardType("SwerveDrive");

        //         builder.addDoubleProperty("Front Left Angle", () -> mSwerveMods[0].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Front Left Velocity", () -> mSwerveMods[0].getState().speedMetersPerSecond, null);

        //         builder.addDoubleProperty("Front Right Angle", () -> mSwerveMods[1].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Front Right Velocity", () -> mSwerveMods[1].getState().speedMetersPerSecond, null);

        //         builder.addDoubleProperty("Back Left Angle", () -> mSwerveMods[2].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Back Left Velocity", () -> mSwerveMods[2].getState().speedMetersPerSecond, null);

        //         builder.addDoubleProperty("Back Right Angle", () -> mSwerveMods[3].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Back Right Velocity", () -> mSwerveMods[3].getState().speedMetersPerSecond, null);

        //         // builder.addDoubleProperty("Robot Angle", () -> Pose.instance != null ? Pose.instance.getGyroYaw().getRadians() : 0.0, null);
        //     }
        // });

        if (Constants.fullDashboard) {
            SmartDashboard.putData(LoggedCommands.runOnce("Sync Swerve to CANcoders", this::resetModulesToAbsolute, this).ignoringDisable(true));

            SmartDashboard.putData("Drive Test", LoggedCommands.sequence("Drive Test",
                LoggedCommands.runOnce("Move forward", () -> drive(new Translation2d(0.25, 0.0).times(SwerveConstants.maxSpeed), 0.0, true), this),
                Commands.waitSeconds(2.0),
                Stop()));
        }
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    Pose.instance.getHeading()
                                );

        driveRobotRelative(desiredChassisSpeeds, isOpenLoop);
    }

    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelativeAuto(ChassisSpeeds desirChassisSpeeds) {
        driveRobotRelative(desirChassisSpeeds, false);
    }

    public void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds, boolean isOpenLoop) {
        ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02);
        lastSpeeds = discretizedSpeeds;
        
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(discretizedSpeeds); 
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        DogLog.log("Swerve/Desired Module States", swerveModuleStates);
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void alignStraight() {
        SwerveModuleState aligned = new SwerveModuleState(0.0, new Rotation2d());

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(aligned, false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setMotorsToCoast() {
        for(SwerveModule mod : mSwerveMods){
            mod.setCoastMode();  
        }
        DogLog.log("Swerve/Status", "Coasted Swerve Motors");
    }

    public void setDriveMotorsToCoast(){
        for(SwerveModule mod : mSwerveMods){
            mod.setDriveCoastMode();  
        }
        DogLog.log("Swerve/Status", "Coasted Swerve Drive Motors");
    }

    public Command CoastDriveMotors() {
        return LoggedCommands.runOnce("Set Swerve Drive to Coast", this::setDriveMotorsToCoast);
    }

    public void setMotorsToBrake(){
        for(SwerveModule mod : mSwerveMods){
            mod.setBrakeMode();  
        }
        DogLog.log("Swerve/Status", "Braked Swerve Motors");
    }

    public void setDriveMotorsToBrake(){
        for(SwerveModule mod : mSwerveMods){
            mod.setDriveBrakeMode();
        }
        DogLog.log("Swerve/Status", "Braked Swerve Drive Motors");
    }

    public Command BrakeDriveMotors() {
        return LoggedCommands.runOnce("Set Swerve Drive to Brake", this::setDriveMotorsToBrake);
    }

    public void stopSwerve(){
        drive(new Translation2d(0, 0), 0, false);
        DogLog.log("Swerve/Status", "Stopped Swerve");
    }

    public Command Stop() {
        return LoggedCommands.runOnce("Stop Swerve", this::stopSwerve, this);
    }

    public void xSwerve() {
        Rotation2d[] rotations = {
            Rotation2d.fromDegrees(45),
            Rotation2d.fromDegrees(-45),
            Rotation2d.fromDegrees(-45),
            Rotation2d.fromDegrees(45),
        };
        
        for (int i = 0; i < mSwerveMods.length; i++) {
            mSwerveMods[i].setDesiredState(new SwerveModuleState(0, rotations[i]), false);
        }
    }

    public Command HoldX() {
        return LoggedCommands.sequence("Hold Swerve X",
            Commands.runOnce(this::xSwerve, this),
            Commands.idle(this)
        );
    }

    // Apply voltage directly to the drive motors
    // Used for SysId characterization
    private void setDriveVolts(Voltage output) {
        for(SwerveModule mod : mSwerveMods){
            mod.setDriveVoltage(output);
        }
    }

    // Apply voltage directly to the primary steering motor
    // Used for SysId characterization
    private void setSteerVolts(Voltage output) {
        SwerveModule mod0 = mSwerveMods[0];

        mod0.setAngleVoltage(output);
    }

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,               // Use default ramp rate (1 V/s)
            Units.Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,                // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setDriveVolts(volts),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,               // Use default ramp rate (1 V/s)
            Units.Volts.of(7), // Use dynamic voltage of 7 V
            null,                // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setSteerVolts(volts),
            null,
            this
        )
    );

    public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineTranslation.quasistatic(direction);
    }

    public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineTranslation.dynamic(direction);
    }

    public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineSteer.quasistatic(direction);
    }

    public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineSteer.dynamic(direction);
    }

    public Translation2d direction() {
        return new Translation2d(lastSpeeds.vxMetersPerSecond, lastSpeeds.vyMetersPerSecond);
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("Swerve/Current Command", currentCommand == null ? "None" : currentCommand.getName());

        boolean aligned = true;
        for(SwerveModule mod : mSwerveMods) {
            DogLog.log("Swerve/Mod/" + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " Alignment Error", mod.alignmentError());
            SmartDashboard.putBoolean("Swerve/Mod/" + mod.moduleNumber + " Aligned", mod.isAligned());
            aligned = aligned && mod.isAligned();
        }
        SmartDashboard.putBoolean("Swerve/Modules Aligned", aligned);
        DogLog.log("Swerve/Actual Module States", getModuleStates());        
    }
}