package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class OTFPathFinding {

    public static void main(String[] args)
    {

        Pose2d[] bluePoseArray = new Pose2d[]
                {
                        new Pose2d(4.080,3.305,new Rotation2d(0)),
                        new Pose2d(3.658,4.033,new Rotation2d(0)),
                        new Pose2d(4.080,4.750,new Rotation2d(0)),
                        new Pose2d(4.902,4.750,new Rotation2d(0)),
                        new Pose2d(5.335,4.033,new Rotation2d(0)),
                        new Pose2d(4.902,3.305,new Rotation2d(0))
                };

        Pose2d[] redPoseArray = new Pose2d[]
                {
                        new Pose2d(13.493,3.305,new Rotation2d(0)),
                        new Pose2d(13.902,4.033,new Rotation2d(0)),
                        new Pose2d(13.493,4.750,new Rotation2d(0)),
                        new Pose2d(12.650,4.750,new Rotation2d(0)),
                        new Pose2d(12.233,4.033,new Rotation2d(0)),
                        new Pose2d(12.650,3.305,new Rotation2d(0))
                };
    }
}
