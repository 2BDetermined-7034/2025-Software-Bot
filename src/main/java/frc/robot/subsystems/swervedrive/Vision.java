package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Vision {

	AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
	PhotonCamera frontCam, backCam;
	Transform3d frontCamToRobot, backCamToRobot;
	PhotonPoseEstimator frontPoseEstimator, backPoseEstimator;

	/**
	 *
	 * @param
	 */
	public Vision() {
		frontCam = new PhotonCamera("front");
		backCam = new PhotonCamera("back");
		frontCamToRobot = new Transform3d(new Translation3d(0.39, 0, 0), new Rotation3d(0,0,0));
		backCamToRobot = new Transform3d(new Translation3d(-0.39, 0.0, 0), new Rotation3d(0,0, Math.PI));
		frontPoseEstimator = new PhotonPoseEstimator(tagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamToRobot);
		backPoseEstimator = new PhotonPoseEstimator(tagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamToRobot);
	}

	public ArrayList<MeasurementWithTimeStamp> updateVisionOdometry() {
		ArrayList<MeasurementWithTimeStamp> measurementWithTimeStamps = new ArrayList<>();
		for(PhotonPipelineResult result : frontCam.getAllUnreadResults()) {
			var estimateOptional = frontPoseEstimator.update(result);
			estimateOptional.ifPresent(estimatedRobotPose ->
					measurementWithTimeStamps.add(
							new MeasurementWithTimeStamp(result.getTimestampSeconds(), estimatedRobotPose)));

		}

		for(PhotonPipelineResult result : backCam.getAllUnreadResults()) {
			var estimateOptional = backPoseEstimator.update(result);
			estimateOptional.ifPresent(estimatedRobotPose ->
					measurementWithTimeStamps.add(
							new MeasurementWithTimeStamp(result.getTimestampSeconds(), estimatedRobotPose)));

		}

		return measurementWithTimeStamps;
	}

}
