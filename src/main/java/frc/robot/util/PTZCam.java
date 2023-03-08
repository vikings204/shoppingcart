package frc.robot.util;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;

@SuppressWarnings("resource")
public class PTZCam {
    public void InitCS() {
        HttpCamera cam = new HttpCamera("Driver Camera", "");
        MjpegServer srv = new MjpegServer("Driver Camera Srv", 1185);
        srv.setSource(cam);
    }
}