package frc.robot.subsystems.qrfc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import java.io.Console;
import java.util.Arrays;

import org.opencv.objdetect.Board;

public class Reef extends SubsystemBase {

    /*
    STEPS:
    1. Know what is new (in the arrays) - DONE
    2. Get rid of old reference of reef status - DONE
    3. Replace w/ new positional copy - DONE
    4. Send back corrected reef status - IN PROGRESS
    5. Store correct pose (be prepared to relay this in the future) - NOT STARTED
     */
    //Each indice in m_reefStatus correlates to its sister pose in the pose array
    private final Pose2d[] reefPoses = new Pose2d[12];
    // Reef tracking is up to you!
    boolean[] m_reefStatus = new boolean[12];

    private boolean[] onboardReefStatus = new boolean[12];

    private int dodecagonLookat;

    public Reef () {
        SmartDashboard.putBooleanArray("Reef", m_reefStatus);
    }


    private Pose2d getReefDestinationPose(int i){
        return reefPoses[i];
    }

    /**
     * Compares local copy and new user input data (a similar boolean array of the same length)
     * and hunts down and returns the <STRONG>indice of where the reef should be green at</STRONG>
     * @param old Cached boolean array of the last known states of the reef GUI
     * @return -1 if userInput has not selected location. Else returns value [0,11].
     */
    public int findNewInputIndice(boolean[] old) {
        boolean[] userInput = SmartDashboard.getBooleanArray("Reef", onboardReefStatus);
        for(int i = 0 ; i <= 11; ++i) {
            if(userInput[i] && !old[i] ) { //true vs true overlap
                return i;
            }
        }
        return -1;

    }

    /**
     * Overwrites onboardReefStatus with an all false status, then replaces the newReefIndex parameter index with true
     * @param newReefIndex index in the array that's going to be true :3c
     */
    public void changeReefPosition(int newReefIndex) {
        Arrays.fill(onboardReefStatus,false); //Clear onboard reef status
        if(newReefIndex != -1){
            onboardReefStatus[newReefIndex] = true;
        }
    }

    /**
     * Called periodically in singleton instance
     */
    @Override
    public void periodic() {
        // int indice = findNewInputIndice(onboardReefStatus);
        // changeReefPosition(indice);
        // SmartDashboard.putBooleanArray("Reef", onboardReefStatus);
        boolean[] newReefStatus = SmartDashboard.getBooleanArray("Reef", m_reefStatus);
        if(newReefStatus == m_reefStatus){
            System.out.println("im gonna crash out");
        }else {
            System.out.println("THey're diffrent!!!@!!!");
        }
    }
}