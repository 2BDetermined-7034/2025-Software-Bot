package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class Controller extends GenericHID {
	enum Type {
		PS5,
		LOGITECH
	}

	public Controller(Type type, int port) {
		super(port);
	}

	public double getLeftX() {
		return super.getRawAxis(0);
	}

	public double getLeftY() {
		return super.getRawAxis(1);
	}

	public double getRightX() {
		return super.getRawAxis(4);
	}

	public double getRightY() {
		return super.getRawAxis(5);
	}
}
