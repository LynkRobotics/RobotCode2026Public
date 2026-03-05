package frc.robot;

import com.ctre.phoenix6.CANBus;

// Single code location to lay out all ports and buses, to ensure no conflicts
public enum Ports {
	PIGEON(0, Bus.SWERVE),
	CANDLE(21, Bus.SWERVE),
	CLIMBER(1, Bus.MECH),
	FLOOR(3, Bus.MECH),
	FLOOR_B(9, Bus.MECH),
	FEEDER(5, Bus.MECH),
	INTAKE_A(7, Bus.RIO),     // Primary
	INTAKE_B(15, Bus.RIO),    // Opposed
	FLYWHEEL_A(10, Bus.MECH), // Primary
	FLYWHEEL_B(11, Bus.MECH), // Aligned
	FLYWHEEL_C(12, Bus.MECH), // Opposed
	FLYWHEEL_D(13, Bus.MECH), // Opposed
	KICKER(14, Bus.MECH);

    public enum Bus {
        RIO("rio"),
        MECH("LynkMechanisms"),
        SWERVE("LynkSwerve");
    
        public final CANBus bus;

        private Bus(String name) {
            this.bus = new CANBus(name);
        }
    }    

	public final int id;
	public final CANBus bus;
	public final String busName;

	private Ports(int id, Bus bus) {
		this.id = id;
		this.bus = bus.bus;
		this.busName = bus.bus.getName();
	}
}