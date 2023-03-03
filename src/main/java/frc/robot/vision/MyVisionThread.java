package frc.robot.vision;

import java.time.Duration;
import java.time.Instant;
import java.util.Map;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cameraserver.CameraServerSharedStore;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class MyVisionThread extends Thread {
    UsbCamera m_camera;
    private MjpegServer m_server;

    Point m_center;


    public void run() {
        System.out.println("Creating new camera");
        m_camera = new UsbCamera("Webcam",0);
        m_camera.setVideoMode(PixelFormat.kYUYV, 640, 480, 10);
        System.out.println("Starting capture");
        m_server = CameraServer.startAutomaticCapture(m_camera);
  

        // Set the resolution
        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Circle", 160, 120);

        

        //MjpegServer m_rawServer = CameraServer.addServer("serve_raw");
        //m_rawServer.setSource(m_camera);        
        
        outputStream.setPixelFormat(PixelFormat.kMJPEG);
        outputStream.setDescription("What does this do?");
        Shuffleboard.getTab("Driving").add(outputStream).withPosition(5,0).withSize(4,4).withProperties(Map.of("Title", "fartknocker"));
        Shuffleboard.update();

        // Mats are very memory expensive. Lets reuse this Mat.
        Mat mat = new Mat();
     
        while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
            } else {
                outputStream.putFrame(mat);
            }
        }
    }
    public  void setCenter(Point center) {
        m_center = center;

    } 

    public  Point getCenter() {
        return m_center;
    }



}
