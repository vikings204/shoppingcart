package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class TagVisionSubsystem extends SubsystemBase {
    private PhotonCamera pv;

    public TagVisionSubsystem() {
        pv = new PhotonCamera("apriltagvision");
        //setDefaultCommand(this.run(() -> System.out.println("TagVisionSubsystem fully inactive!!!")));
    }

    /*@Override
    public void periodic() {
        // runs every scheduler loop (every ~20ms)
        PhotonPipelineResult r = pv.getLatestResult();
        System.out.println("PV-AT_ID: " + r.getBestTarget().getFiducialId());
    }*/

    public String printLatestID() {
        PhotonPipelineResult r = pv.getLatestResult();
        if (r.hasTargets()) {
            return "PV-AT_ID: " + r.getBestTarget().getFiducialId();
        } else {
            return "PV-AT-NULL";
        }
    }
}
