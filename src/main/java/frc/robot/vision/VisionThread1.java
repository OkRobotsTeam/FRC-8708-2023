package frc.robot.vision;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class VisionThread1 extends Thread {
    UsbCamera m_camera1;
    private MjpegServer m_server1;

    public boolean m_webcamPresent;

    public void run() {
        System.out.println("Connecting to camera 1");
        m_camera1 = new UsbCamera("Webcam1", 0);
        m_camera1.setVideoMode(PixelFormat.kYUYV, 320, 240, 6);
        System.out.println("Starting frame capture on camera 1");
        m_server1 = CameraServer.startAutomaticCapture(m_camera1);

        // Reduce the impact on bandwidth
        m_server1.setCompression(50);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink1 = CameraServer.getVideo();

        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream1 = CameraServer.putVideo("Camera1", 320, 240);

        outputStream1.setPixelFormat(PixelFormat.kMJPEG);
        Shuffleboard.getTab("Driving").add(outputStream1).withPosition(6, 0).withSize(4, 4);
        Shuffleboard.update();

        Mat mat = new Mat();

        while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink1.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream1.notifyError(cvSink1.getError());
                continue;
            } else {
                outputStream1.putFrame(mat);
            }
        }
    }
}