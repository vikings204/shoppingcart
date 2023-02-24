package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class TagVisionSubsystem extends SubsystemBase {
    private PhotonCamera pv;

    public TagVisionSubsystem() {
        pv = new PhotonCamera("apriltagvision");
        setDefaultCommand(this.run(() -> System.out.println("TagVisionSubsystem fully inactive!!!")));
    }

    @Override
    public void periodic() {
        // runs every scheduler loop (every ~20ms)
        pv.getLatestResult();
    }
}
