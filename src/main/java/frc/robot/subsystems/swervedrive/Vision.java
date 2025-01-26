package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;

public class Vision {

	AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
	PhotonCamera frontCam, backCam;
	Transform3d robotToFront, robotToBack;
	PhotonPoseEstimator frontPoseEstimator, backPoseEstimator;
	final Matrix<N3, N1> stdDevs;

	/**
	 *
	 * @param
	 */
	public Vision() {
		frontCam = new PhotonCamera("front");
		backCam = new PhotonCamera("back");
		robotToFront = new Transform3d(new Translation3d(0.39, 0, 0), new Rotation3d(0,0,0));
		robotToBack = new Transform3d(new Translation3d(-0.39, 0.0, 0), new Rotation3d(0,0, Math.PI));
		frontPoseEstimator = new PhotonPoseEstimator(tagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFront);
		backPoseEstimator = new PhotonPoseEstimator(tagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBack);
		stdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 2.0, 2.0, 0.5);
	}

	public ArrayList<MeasurementWithTimeStamp> updateVisionOdometry() {
		ArrayList<MeasurementWithTimeStamp> measurementWithTimeStamps = new ArrayList<>();

		for(PhotonPipelineResult result : frontCam.getAllUnreadResults()) {
			var estimateOptional = frontPoseEstimator.update(result);
			estimateOptional.ifPresent(estimatedRobotPose ->
					measurementWithTimeStamps.add(
							new MeasurementWithTimeStamp(result.getTimestampSeconds(), estimatedRobotPose, stdDevs)));

		}

		for(PhotonPipelineResult result : backCam.getAllUnreadResults()) {
			var estimateOptional = backPoseEstimator.update(result);
			estimateOptional.ifPresent(estimatedRobotPose ->
					measurementWithTimeStamps.add(
							new MeasurementWithTimeStamp(result.getTimestampSeconds(), estimatedRobotPose, stdDevs)));

		}

		return measurementWithTimeStamps;
	}

}
