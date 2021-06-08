package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSource;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Point;
import java.util.*;

/** This is now fossil code. It does nothing, but later I will refurbish it to help with autonomous navigation. */
class PathDrawer
{
    static final String kResetPath = "PathDrawer/Reset Path";

    VisionHandler visionHandler;

    CameraServer camServer = CameraServer.getInstance();
    CvSource source;
    final int imageWidth = 388;
    final int imageHeight = 320;

    final Scalar pathColor = new Scalar(0, 255, 255);
    /**Pixel coord. of top left of team's area */
    Point fieldPixMin;
    /**Pixel coord. of bottom right of team's area */
    Point fieldPixMax;
    /**
     * How many pixels in the x and y directions are equivalent to one square meter
     * The two axes are distinct because the field image may not be resized to its original aspect ratio.
    */
    Point pixelsPerMeter;
    
    List<Point> points;
    Mat fieldImage = new Mat();
    Mat finalImage = new Mat();

    double timer;

    PathDrawer(String streamName, VisionHandler visionHandler)
    {
        this.visionHandler = visionHandler;
        //navxSensor = visionHandler.robot.driveSystem.navxSensor;

        //Initialize Dashboard stuff
        SmartDashboard.putBoolean(kResetPath, false);

        source = camServer.putVideo(streamName, imageWidth, imageHeight);
        source.setFPS(VisionHandler.FPS);

        points = new ArrayList<Point>();

        //Load and scale field image
        Mat unscaledFieldImage = Imgcodecs.imread(Filesystem.getDeployDirectory() + "/2019-half-field.jpg");
        Imgproc.resize(unscaledFieldImage, fieldImage, new Size(imageWidth, imageHeight));
        
        final Point fieldResizeRatio = new Point((double)imageWidth / (double)unscaledFieldImage.width(), (double)imageHeight / (double)unscaledFieldImage.height());
        fieldPixMin = new Point(217 * fieldResizeRatio.x, 42 * fieldResizeRatio.y);
        fieldPixMax = new Point(796 * fieldResizeRatio.x, 615 * fieldResizeRatio.y);
        pixelsPerMeter = new Point((fieldPixMax.x - fieldPixMin.x) / Field.teamSectionSize.x,
            (fieldPixMax.y - fieldPixMin.y) / Field.teamSectionSize.y);
    }
    public void Update(double deltaTime)
    {
        timer += deltaTime;

        fieldImage.copyTo(finalImage); //Draw field

        /*
        if (SmartDashboard.getBoolean(kResetPath, false))
        {
            points.clear();
            SmartDashboard.putBoolean(kResetPath, false);
        }

        //Each frame, add the current location to the list of all previous locations
        Point p = new Point();
        //The sensor's "displacement" is relative to the robot's starting position and orientation.
        //For now, we assume the robot starts at the exact center of the level 1 hab platform (including ramp)
        //And is facing towards the cargo ship.
        Waypoint hab1Center = visionHandler.robot.autonomous.waypointLattice.targets.get("Hab 1 Center");
        p.x = fieldPixMin.x + (navxSensor.getDisplacementX() + hab1Center.position.x) * pixelsPerMeter.x;
        p.y = fieldPixMin.y + (navxSensor.getDisplacementY() + hab1Center.position.y) * pixelsPerMeter.y;
        points.add(p);
        for (int i = 1; i < points.size(); i++)
        {
            //Starting at the second location, draw a line from each location to the previous location
            Imgproc.line(finalImage, points.get(i-1), points.get(i), pathColor);
        }*/

        /* Wiggly rectangle for testing purposes.
        int x = (int) (Math.sin(timer) * 64.0);
        Imgproc.rectangle(finalImage, new Point(64 + x, 64), new Point(128 + x, 128), new Scalar(0, 255, 255), 4);
        */
        source.putFrame(finalImage); //Output to camera stream
    }
}