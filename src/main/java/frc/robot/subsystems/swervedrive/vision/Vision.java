package frc.robot.subsystems.swervedrive.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Vision {

	private static final AprilTagFieldLayout k2025Reefscape = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

	/**
	 * Construct a vision singleton
	 */
	public Vision() {

	}

	/**
	 * create a list of drivebase {@link VisionPoseMeasurement} estimates from vision cameras.
	 *
	 * @return the list of (pose measurements, timestamps, and stdDevs) to feed to the {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
	 */
	public ArrayList<VisionPoseMeasurement> getVisionPoseMeasurements() {
		ArrayList<VisionPoseMeasurement> measurementWithTimeStamps = new ArrayList<>();

		for (Cameras camera : Cameras.values()) {

			for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
				var estimateOptional = camera.update(result);
				camera.updateEstimationStdDevs(estimateOptional, result.getTargets());
				estimateOptional.ifPresent(estimatedRobotPose ->
						measurementWithTimeStamps.add(
								new VisionPoseMeasurement(
										result.getTimestampSeconds(), estimatedRobotPose, camera.getCurrentStdDevs())));

			}
		}

		return measurementWithTimeStamps;
	}

	enum Cameras {
		FRONT_CAM("front",
				new Transform3d(
						new Translation3d(0.39, 0, 0.135),
						new Rotation3d(0, Math.toRadians(-30), 0)),
				PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				VecBuilder.fill(2.0, 2.0, 0.5),
				VecBuilder.fill(1.0, 1.0, 0.5)),
		BACK_CAM("back",
				new Transform3d(
						new Translation3d(-0.39, 0.0, 0.23),
						new Rotation3d(0, 0, Math.PI)),
				PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				VecBuilder.fill(2.0, 2.0, 0.5),
				VecBuilder.fill(1.0, 1.0, 0.5));

		private final PhotonCamera photonCamera;
		private final Transform3d robotToCam;
		private final PhotonPoseEstimator poseEstimator;
		private Vector<N3> singleTagStdDevs;
		private Vector<N3> multiTagStdDevs;


		private Vector<N3> curStdDevs;


		/**
		 * Vision Camera constructor
		 * create one for each camera on the robot
		 *
		 * @param name             name of the camera in photonvision
		 * @param robotToCam       robotToCamera Transform
		 * @param poseStrategy     strategy to be used by the photonPoseEstimator
		 * @param singleTagStdDevs standard deviations for when a single tag is visible
		 * @param multiTagStdDevs  standard deviation for when multiple tags are visible
		 */
		Cameras(String name, Transform3d robotToCam, PhotonPoseEstimator.PoseStrategy poseStrategy, Vector<N3> singleTagStdDevs, Vector<N3> multiTagStdDevs) {
			this.photonCamera = new PhotonCamera(name);
			this.robotToCam = robotToCam;
			this.singleTagStdDevs = singleTagStdDevs;
			this.multiTagStdDevs = multiTagStdDevs;
			this.poseEstimator = new PhotonPoseEstimator(k2025Reefscape, poseStrategy, robotToCam);
			this.curStdDevs = singleTagStdDevs;
		}

		public PhotonCamera getPhotonCamera() {
			return photonCamera;
		}

		public Transform3d getRobotToCam() {
			return robotToCam;
		}

		public PhotonPoseEstimator getPoseEstimator() {
			return poseEstimator;
		}

		public Vector<N3> getCurrentStdDevs() {
			return curStdDevs;
		}

		public Vector<N3> getSingleTagStdDevs() {
			return singleTagStdDevs;
		}

		public void setSingleTagStdDevs(Vector<N3> singleTagStdDevs) {
			this.singleTagStdDevs = singleTagStdDevs;
		}

		public Vector<N3> getMultiTagStdDevs() {
			return multiTagStdDevs;
		}

		public void setMultiTagStdDevs(Vector<N3> multiTagStdDevs) {
			this.multiTagStdDevs = multiTagStdDevs;
		}

		/**
		 * @see PhotonCamera#getAllUnreadResults()
		 *
		 * @return
		 */
		public List<PhotonPipelineResult> getAllUnreadResults() {
			return photonCamera.getAllUnreadResults();
		}

		/**
		 * @see PhotonPoseEstimator#update(PhotonPipelineResult)
		 *
		 * @param result
		 * @return
		 */
		public Optional<EstimatedRobotPose> update(PhotonPipelineResult result) {
			return poseEstimator.update(result);
		}

		/**
		 * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
		 * on number of tags, estimation strategy, and distance from the tags.
		 *
		 * @param estimatedPose The estimated pose to guess standard deviations for.
		 * @param targets       All targets in this camera frame
		 */
		private void updateEstimationStdDevs(
				Optional<EstimatedRobotPose> estimatedPose,
				List<PhotonTrackedTarget> targets) {
			if (estimatedPose.isEmpty()) {
				curStdDevs = singleTagStdDevs;
				return;
			}
			Vector<N3> estStdDevs = singleTagStdDevs;
			int numTags = 0;
			double avgDistance = 0;

			for (PhotonTrackedTarget target : targets) {
				Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
				if (tagPose.isEmpty()) {
					continue;
				}
				numTags++;
				avgDistance += tagPose.get().toPose2d().getTranslation().getDistance(
						estimatedPose.get().estimatedPose.toPose2d().getTranslation());
			}

			if (numTags == 0) {
				curStdDevs = singleTagStdDevs;
				return;
			}

			avgDistance /= numTags;
			if (numTags > 1) {
				estStdDevs = multiTagStdDevs;
			}
			// Increase std devs based on (average) distance
			if (numTags == 1 && avgDistance > 4) {
				estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
			} else {
				estStdDevs = estStdDevs.times(1 + (avgDistance * avgDistance / 30));
			}
			curStdDevs = estStdDevs;

		}
	}
}
