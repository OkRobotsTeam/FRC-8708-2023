package frc.robot.vision;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class VisionThread2 extends Thread {
    UsbCamera m_camera2;
    private MjpegServer m_server2;

    public boolean m_webcamPresent;

    public void run() {
        System.out.println("Connecting to camera 2");
        m_camera2 = new UsbCamera("Webcam2", 1);
        m_camera2.setVideoMode(PixelFormat.kYUYV, 320, 240, 6);
        System.out.println("Starting frame capture on camera 2");
        m_server2 = CameraServer.startAutomaticCapture(m_camera2);

        // Reduce the impact on bandwidth
        m_server2.setCompression(50);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink2 = CameraServer.getVideo();

        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream2 = CameraServer.putVideo("Camera2", 320, 240);

        outputStream2.setPixelFormat(PixelFormat.kMJPEG);
        Shuffleboard.getTab("Driving").add(outputStream2).withPosition(6, 0).withSize(4, 4);
        Shuffleboard.update();

        Mat mat = new Mat();

        while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink2.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream2.notifyError(cvSink2.getError());
                continue;
            } else {
                outputStream2.putFrame(mat);
            }
        }
    }
}