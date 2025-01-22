package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

public class Vision {

	AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
	PhotonCamera centerCam = new PhotonCamera("center");
	Transform3d centerCamToRobot = new Transform3d(new Translation3d(0.39, 0, 0), new Rotation3d(0,0,0));
	PhotonPoseEstimator centerPoseEstimator = new PhotonPoseEstimator(tagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, centerCamToRobot);

	/**
	 *
	 * @param
	 */
	public Vision() {
	}

	public ArrayList<MeasurementWithTimeStamp> updateVisionOdometry() {
		List<PhotonPipelineResult> centerPipelineResult = centerCam.getAllUnreadResults();
		ArrayList<MeasurementWithTimeStamp> measurementWithTimeStamps = new ArrayList<>();
		for(PhotonPipelineResult result : centerPipelineResult) {
			var estimateOptional = centerPoseEstimator.update(result);
			estimateOptional.ifPresent(estimatedRobotPose ->
					measurementWithTimeStamps.add(
							new MeasurementWithTimeStamp(result.getTimestampSeconds(), estimatedRobotPose)));

		}

		return measurementWithTimeStamps;



	}

}
