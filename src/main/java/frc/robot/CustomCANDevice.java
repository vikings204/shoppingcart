package frc.robot;

import edu.wpi.first.hal.can.CANJNI;

import java.lang.reflect.Type;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

// https://www.chiefdelphi.com/t/can-bus-java-documentation-with-custom-sensors/140503
public class CustomCANDevice {
    /* Check the datasheets for your device for the arbitration IDs of the
        messages you want to send.  By convention, this is a bitstring
        containing the model number, manufacturer number, and api number. */
    private final int SEND_ARB_ID;
    private final int RECV_ARB_ID;
    //private final Type RECV_DECODE_TYPE;

    /*  Device ID, from 0 to 63. */
    private final int DEVICE_NUMBER;

    private final IntBuffer status = ByteBuffer.allocateDirect(4).asIntBuffer();
    private final IntBuffer messageId = ByteBuffer.allocateDirect(4).asIntBuffer();
    private final ByteBuffer timestamp = ByteBuffer.allocate(4);

    private final int READ_DELAY = 10; // milliseconds between checking for another packet
    private final int READ_TIMEOUT = 100; // amount of times to check a packet and wait READ_DELAY ms

    public CustomCANDevice(int id, int send_arb_id, int recv_arb_id/*, Type recv_type*/) {
        this.DEVICE_NUMBER = id;
        this.SEND_ARB_ID = send_arb_id;
        this.RECV_ARB_ID = recv_arb_id;
        //this.RECV_DECODE_TYPE = recv_type;
    }

    private String getLatestPacket() {
        /* To receive a message, put the message ID you're looking for in this
            buffer.  CANJNI...ReceiveMessage  will not block waiting for it,
            but just return null if it hasn't been received yet. */
        messageId.clear();
        messageId.put(0, RECV_ARB_ID | DEVICE_NUMBER);

        status.clear();
        //byte[] d =
        return Arrays.toString(CANJNI.FRCNetCommCANSessionMuxReceiveMessage(
                messageId,
                0x1FFFFFFF, // 29 1's, ensures that only packets that have the exact message id are captured
                timestamp
        ));
        /*
        //CANStatus s = new CANStatus();
        //CANJNI.getCANStatus(s);

        if (d != null) {
            //CANExceptionFactory.checkStatus(s., MESSAGE1_ARB_ID);
            System.out.println("Received a message: " + Arrays.toString(d));
        }*/
    }

    @SuppressWarnings("ConstantValue")
    public String readNext() {
        for (int i = 0; i < READ_TIMEOUT; i++) {
            String p = getLatestPacket();
            if (p != null) {
                return p;
            } else {
                try {
                    TimeUnit.MILLISECONDS.sleep(READ_DELAY);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        return null;
    }

    public void sendNext(String d) {
        CANJNI.FRCNetCommCANSessionMuxSendMessage(
                SEND_ARB_ID,
                d.getBytes(StandardCharsets.UTF_8),
                CANJNI.CAN_SEND_PERIOD_NO_REPEAT
        );
    }
}
