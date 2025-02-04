package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

import javax.naming.ldap.Control;

public class Controller extends GenericHID {
	enum Type {
		PS5,
		LOGITECH
	}

	private final Type type;

	public Controller(Type type, int port) {
		super(port);
		this.type = type;
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
