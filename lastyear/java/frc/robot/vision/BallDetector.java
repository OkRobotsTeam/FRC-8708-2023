package frc.robot.vision;

  
import org.opencv.core.Core;
import org.opencv.core.Mat;

import org.opencv.core.Point;

import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.DriverStation;

import java.awt.Robot;



public class BallDetector {
 
    Point center = new Point(80, 60);

    /**
     *
     * @param width The width of the image (check your camera)
     */
    Mat small = new Mat();
    Mat gray = new Mat();
    boolean blue = false;
    /*
    Mat hls = new Mat();
    Mat mask = new Mat(); 
    Mat combined = new Mat();
    */
    Size resolution = new Size(160,120);
    
    public BallDetector() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            blue = true;
        }
    } 

    public Mat processFrame(Mat input, int color) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV

        // if something is wrong, we assume there's no ball
        if (input.empty()) {
            return input;
        }

        
        
        Imgproc.resize(input,small,resolution,0,0,Imgproc.INTER_CUBIC);


        /* Filter out non-blue pixels.  
           Remvoing because it doesn't seem to make circle detection better

        Imgproc.cvtColor(small, hls, Imgproc.COLOR_BGR2HLS);
        Core.inRange(hls, new Scalar(80, 70, 0), new Scalar(115, 255, 255), mask);
        combined.copySize(hls);
        Core.bitwise_and(small,small,combined,mask);
        Mat gray = new Mat();
        
        */
        //Imgproc.cvtColor(small,gray,Imgproc.COLOR_RGB2GRAY);
        if (blue) {
            Core.extractChannel(small,gray,0);
        } else {
            Core.extractChannel(small,gray,1);
        }
        Imgproc.medianBlur(gray, gray, 11);
        Mat circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                (double)gray.rows()/.0000001, // change this value to detect circles with different distances to each other
                100.0, 15.0, 10, 20); // change the last two parameters
                // (min_radius & max_radius) to detect larger circles


        Mat output = small;
        //Draw Circles on image
        for (int x = 0; x < circles.cols(); x++) {
            double[] c = circles.get(0, x);
            this.center = new Point(Math.round(c[0]), Math.round(c[1]));
            // circle center
            Imgproc.circle(small, center, 1, new Scalar(255,0,0), 3, 8, 0 );
            // circle outline
            int radius = (int) Math.round(c[2]);
            Imgproc.circle(small, center, radius, new Scalar(255,0,255), 3, 8, 0 );
            //System.out.printf("Circle found: x:%d y:%d  radius:%d\n", Math.round(c[0]) , Math.round(c[1]), Math.round(c[2]) );
        }



        return small; // return the mat with rectangles drawn
    }




}
