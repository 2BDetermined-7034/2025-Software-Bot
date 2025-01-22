package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.EstimatedRobotPose;

public class MeasurementWithTimeStamp {
	private double timeStamp;
	private EstimatedRobotPose robotPose;
	public MeasurementWithTimeStamp(double timeStamp, EstimatedRobotPose robotPose) {
		this.timeStamp = timeStamp;
		this.robotPose = robotPose;
	}

	public double getTimeStamp() {
		return timeStamp;
	}

	public Pose2d getRobotPose() {
		return robotPose.estimatedPose.toPose2d();
	}
}
