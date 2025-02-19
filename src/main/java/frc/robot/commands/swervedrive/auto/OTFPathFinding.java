package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.dyn4j.geometry.Rotation;

public class OTFPathFinding {

	//edit to the positions that we want the robot to go to
	//They are currently the positions of the april tags

	private static final Pose2d[] bluePoseArray = new Pose2d[]
		{
			new Pose2d(3.920,3.305,new Rotation2d(240)),
			new Pose2d(3.458,4.967,new Rotation2d(180)),
			new Pose2d(4.920,4.750,new Rotation2d(120)),
			new Pose2d(4.902,4.950,new Rotation2d(60)),
			new Pose2d(5.535,4.233,new Rotation2d(0)),
			new Pose2d(5.102,3.305,new Rotation2d(300)),

			new Pose2d(4.080,3.105,new Rotation2d(240)),
			new Pose2d(3.258,4.233,new Rotation2d(180)),
			new Pose2d(4.080,4.950,new Rotation2d(120)),
			new Pose2d(5.102,4.750,new Rotation2d(60)),
			new Pose2d(5.535,4.967,new Rotation2d(0)),
			new Pose2d(4.902,3.105,new Rotation2d(300))
		};

	private static final Pose2d[] redPoseArray = new Pose2d[]
		{
			new Pose2d(13.693,3.305,new Rotation2d(240)),
			new Pose2d(14.102,4.233,new Rotation2d(180)),
			new Pose2d(13.493,4.950,new Rotation2d(120)),
			new Pose2d(12.450,4.950,new Rotation2d(60)),
			new Pose2d(12.033,4.967,new Rotation2d(0)),
			new Pose2d(12.650,3.105,new Rotation2d(300)),

			new Pose2d(13.493,3.205,new Rotation2d(240)),
			new Pose2d(14.102,3.967,new Rotation2d(180)),
			new Pose2d(13.893,4.750,new Rotation2d(120)),
			new Pose2d(12.650,4.950,new Rotation2d(60)),
			new Pose2d(12.033,4.233,new Rotation2d(0)),
			new Pose2d(12.450,3.305,new Rotation2d(300))
		};

	private static final Translation2d redReef  = new Translation2d(Units.inchesToMeters(514.13),  Units.inchesToMeters(158.5));
	private static final Translation2d blueReef = new Translation2d(Units.inchesToMeters(176.745), Units.inchesToMeters(158.5));

	public static Command goToPose(SwerveSubsystem swerve, int scoreLocation) {
		Alliance alliance = DriverStation.getAlliance().get();
		if(alliance == Alliance.Blue) {
			return swerve.driveToPose(bluePoseArray[scoreLocation]);
		} else{
			return swerve.driveToPose(redPoseArray[scoreLocation]);
		}
	}

	private static double incTrunc(double x) {
		return x > 0 ? Math.ceil(x) : Math.floor(x);
	}

	public static Command goToNearestReef(SwerveSubsystem swerveSubsystem, boolean isLeft) {
		boolean isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);

		Translation2d reef = isBlue ? blueReef : redReef;

		double angle = swerveSubsystem.getPose().getTranslation().minus(reef).getAngle().getRotations();
		SmartDashboard.putNumber("baseAngle to Reef", angle);
		double clampedAngle = (angle);
		clampedAngle = Units.rotationsToRadians(Math.round(angle * 6.0) / 6.0);

		Translation2d tangent = new Translation2d(Units.inchesToMeters(7) * -Math.sin(clampedAngle), Units.inchesToMeters(7) * Math.cos(clampedAngle)).times(isLeft ? 1 : -1);
		Pose2d clampedPose = new Pose2d(
			reef.plus(new Translation2d(2.0 * Math.cos(clampedAngle), 2.0 * Math.sin(clampedAngle))).plus(tangent),
			Rotation2d.fromRadians(clampedAngle + Math.PI)
		);

		SmartDashboard.putNumber("Target angle", Units.radiansToDegrees(clampedAngle));

		SmartDashboard.putNumberArray("Target pose", new double[] {
			clampedPose.getX(),
			clampedPose.getY(),
			clampedPose.getRotation().getDegrees()
		});

		return Commands.none();
	}
}
