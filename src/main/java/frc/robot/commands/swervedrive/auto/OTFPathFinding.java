package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class OTFPathFinding {

        //edit to the positions that we want the robot to go to
        //They are currently the positions of the april tags

        private Pose2d[] bluePoseArray = new Pose2d[]
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

        private Pose2d[] redPoseArray = new Pose2d[]
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

    public Command goToPos(SwerveSubsystem swerve, int scoreLocation)
    {
        Alliance alliance = DriverStation.getAlliance().get();
        if(alliance == Alliance.Blue)
        {
                return swerve.driveToPose(bluePoseArray[scoreLocation]);
        }else{
                return swerve.driveToPose(redPoseArray[scoreLocation]);
        }
    }
}
