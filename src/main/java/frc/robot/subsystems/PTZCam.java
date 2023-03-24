package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants204;

import java.io.DataOutputStream;
import java.net.HttpURLConnection;
import java.net.URL;

@SuppressWarnings("resource")
public class PTZCam extends SubsystemBase {
    public Joystick js = new Joystick(Constants204.Controller.PTZ_JOYSTICK_PORT);
    public final double deadband = 0.25;
    public final int channel = 0;
    public final int speed = 1;
    public int sendPTZControlCmd(String action, int ch, String code, int arg2) {
        HttpURLConnection connection = null;

        try {
            String query = String.format("action=%s&ch=%s&code=%s&arg1=0&arg2=%s&arg3=0", action, ch, code, arg2);
            URL url = new URL(Constants204.Controller.PTZ_HOSTNAME + "/cgi-bin/ptz.cgi" + "?" + query);
            connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.setRequestProperty("Authorization", "Basic YWRtaW46dGVhbTIwNGZyYw==");

            DataOutputStream wr = new DataOutputStream(connection.getOutputStream());
            wr.close();

            return connection.getResponseCode();
        } catch (Exception e) {
            e.printStackTrace();
            return 0;
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
    }

    @Override
    public void periodic() {
        // runs every scheduler loop (every ~20ms)
        if (js.getTrigger()) {
            double x = Math.abs(js.getX()) < deadband ? 0.0 : js.getX();
            double y = Math.abs(js.getY()) < deadband ? 0.0 : js.getY();
            double z = Math.abs(js.getZ()) < deadband ? 0.0 : js.getZ();

            System.out.println("PTZ CONTROL x=" + x + " y=" + y + " z=" + z);

            if (x > 0) {
                sendPTZControlCmd("start", channel, "Right", speed);
            } else if (x < 0) {
                sendPTZControlCmd("start", channel, "Left", speed);
            } else {
                sendPTZControlCmd("stop", channel, "Right", speed);
                sendPTZControlCmd("stop", channel, "Left", speed);
            }
            if (y > 0) {
                sendPTZControlCmd("start", channel, "Up", speed);
            } else if (y < 0) {
                sendPTZControlCmd("start", channel, "Down", speed);
            } else {
                sendPTZControlCmd("stop", channel, "Up", speed);
                sendPTZControlCmd("stop", channel, "Down", speed);
            }
            if (z > 0) {
                sendPTZControlCmd("start", channel, "ZoomTele", speed);
            } else if (z < 0) {
                sendPTZControlCmd("start", channel, "ZoomWide", speed);
            } else {
                sendPTZControlCmd("stop", channel, "ZoomTele", speed);
                sendPTZControlCmd("stop", channel, "ZoomWide", speed);
            }
        }
    }
}