package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import static frc.robot.Constants204.Vision.*;

import java.util.Objects;

public class TagVisionSubsystem extends SubsystemBase {
    private PhotonCamera pv;
    private int lcID, mcID, rcID, dsID; // leftcubeID, middlecubeID, rightcubeID, doublesubstationID
    private String currentAlliance = "red";

    public TagVisionSubsystem() {
        pv = new PhotonCamera(PHOTONVISION_NAME);

        pv.setDriverMode(false);
        pv.setPipelineIndex(0);
        switchAlliance(); // sets to blue
    }

    public String switchAlliance() {
        if (currentAlliance.equals("blue")) {
            // set to red AT ids
            lcID = 3;
            mcID = 2;
            rcID = 1;
            dsID = 5;
            currentAlliance = "red";
        } else if (currentAlliance.equals("red")) {
            // set to blue AT ids
            lcID = 8;
            mcID = 7;
            rcID = 6;
            dsID = 4;
            currentAlliance = "blue";
        } else {
            System.out.println("VISION ERROR: ALLIANCE STRING FUCKED UP - " + currentAlliance);
        }
        return currentAlliance;
    }

    @Override
    public void periodic() {
        // runs every scheduler loop (every ~20ms)
        System.out.println("PV-AT_ID: " + getLatestID());
    }

    public String getLatestID() {
        PhotonPipelineResult r = pv.getLatestResult();
        if (r.hasTargets()) {
            return "PV-AT_ID: " + r.getBestTarget().getFiducialId();
        } else {
            return "PV-AT-NULL";
        }
    }
}
