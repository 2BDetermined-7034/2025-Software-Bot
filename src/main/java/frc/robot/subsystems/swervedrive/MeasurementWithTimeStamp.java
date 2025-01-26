package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;

public class MeasurementWithTimeStamp {
	private double timeStamp;
	private EstimatedRobotPose robotPose;
	private Matrix<N3, N1> stdDevs;
	public MeasurementWithTimeStamp(double timeStamp, EstimatedRobotPose robotPose, Matrix<N3, N1> stdDevs) {
		this.timeStamp = timeStamp;
		this.robotPose = robotPose;
		this.stdDevs = stdDevs;
	}

	public double getTimeStamp() {
		return timeStamp;
	}

	public Pose2d getRobotPose() {
		return robotPose.estimatedPose.toPose2d();
	}

	public Matrix<N3, N1> getStdDevs() {
		return stdDevs;
	}
}
