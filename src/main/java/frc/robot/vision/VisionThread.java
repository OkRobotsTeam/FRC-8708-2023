package frc.robot.vision;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class VisionThread extends Thread {
    UsbCamera m_camera;
    private MjpegServer m_server;

    public boolean m_webcamPresent;

    public void run() {
        System.out.println("Connecting to camera");
        m_camera = new UsbCamera("Webcam", 0);
        m_camera.setVideoMode(PixelFormat.kYUYV, 640, 480, 10);
        System.out.println("Starting frame capture");
        m_server = CameraServer.startAutomaticCapture(m_camera);

        // Reduce the impact on bandwidth
        m_server.setFPS(10);
        m_server.setCompression(25);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();

        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Camera", 160, 120);

        outputStream.setPixelFormat(PixelFormat.kMJPEG);
        Shuffleboard.getTab("Driving").add(outputStream).withPosition(5, 0).withSize(4, 4);
        Shuffleboard.update();

        Mat mat = new Mat();

        while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                continue;
            } else {
                outputStream.putFrame(mat);
            }
        }
    }
}
