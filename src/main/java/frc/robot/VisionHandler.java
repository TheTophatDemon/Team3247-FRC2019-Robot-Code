package frc.robot;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.PriorityQueue;

//TODO: Test target pairing
//TODO: Test target angle finding
//TODO: Geoffrey Cam
/** Manages the Vision thread and updates the Camera Sources */
class VisionHandler implements Runnable
{
    public static final int WIDTH = 320;
    public static final int HEIGHT = 240;
    /**Frames per second of camera */
    public static final int FPS = 30;
    /**Horizontal field of view of camera */
    public static final double FOV_H = 45.0;
    /**Vertical field of view of camera */
    public static final double FOV_V = 40.0;
    /**Focal length of camera (in pixel units) */
    public static final double FOCAL_LEN = WIDTH / Math.tan(Math.toRadians(FOV_H) / 2.0);

    
    /**Thread used for displaying camera footage & etc. */
    public Thread thread;
    /** Global time in seconds */
    public double time;

    Robot robot;
    //PathDrawer pathDrawer;
    UsbCamera camera;
    /**Camera stream output to the dashboard */
    CvSource cameraSource;
    /**Raw camera stream from the camera on the robot */
    CvSink cameraSink;
    /**OpenCV image object for storing the stream */
    Mat cameraInput = new Mat(WIDTH, HEIGHT, CvType.CV_8UC3);
    /**OpenCV image object for storing the modified stream */
    Mat cameraOutput = new Mat(WIDTH, HEIGHT, CvType.CV_8UC1);

    List<MatOfPoint> contours;

    public class TargetPair
    {
        RotatedRect left;
        RotatedRect right;
        Vector2 center;
        public TargetPair(RotatedRect left, RotatedRect right)
        {
            this.left = left;
            this.right = right;
            center = new Vector2(left.center.x + right.center.x / 2.0,
                left.center.y + right.center.y / 2.0);
        }
        public RotatedRect GetLeft() { return left; }
        public RotatedRect GetRight() { return right; }
        public Vector2 GetCenter() { return center; };
    }
    List<TargetPair> targetPairs = new ArrayList<TargetPair>();
    /**This is true when new vision data has been processed and the target data needs to be recalculated */
    boolean outdatedPairs = false;

    VisionHandler(Robot robot)
    {
        this.robot = robot;
        //pathDrawer = new PathDrawer("path", this);

        //Run the vision pipeline, using a lambda expression to transport the resulting contours for processing
        new VisionThread(camera, new VisionTargetPipeline(), pipeline->{
            contours = pipeline.filterContoursOutput();
            outdatedPairs = true;
        });

        thread = new Thread(this);
        thread.start();
    }
    public void run()
    {
        camera = CameraServer.getInstance().startAutomaticCapture(0);
        cameraSink = CameraServer.getInstance().getVideo(camera);
        cameraSource = new CvSource("White Diamond Vision", VideoMode.PixelFormat.kMJPEG, WIDTH, HEIGHT, FPS);
        CameraServer.getInstance().startAutomaticCapture(cameraSource);

        //long lastTime = System.currentTimeMillis();
        while (!Thread.interrupted())
        {
            /*long now = System.currentTimeMillis();
            double deltaTime = (now - lastTime) / 1000.0;
            if (deltaTime > 1.0 / frameRate)
            {
                time += deltaTime;
                if (robot.isEnabled())
                {
                    pathDrawer.Update(deltaTime);
                }
                lastTime = now;
            }*/

            cameraSink.grabFrame(cameraInput);
            Imgproc.cvtColor(cameraInput, cameraOutput, Imgproc.COLOR_BGR2GRAY);

            //Draw rectangles around target pairs. For debug only. Remove this later.
            List<TargetPair> pairs = GetTargetPairs();
            for (int i = 0; i < pairs.size(); i++)
            {
                TargetPair pair = pairs.get(i);
                Imgproc.rectangle(cameraOutput, pair.left.boundingRect().tl(), pair.right.boundingRect().br(), new Scalar(1.0, 1.0, 1.0));
            }
            
            cameraSource.putFrame(cameraOutput);
        }
    }
    public List<TargetPair> GetTargetPairs()
    {
        if (outdatedPairs)
        {
            /**This list automatically sorts itself from left to right */
            PriorityQueue<RotatedRect> rects = new PriorityQueue<RotatedRect>(new Comparator<RotatedRect>(){
                @Override
                public int compare(RotatedRect a, RotatedRect b)
                {
                    return (int)Math.signum(a.center.x - b.center.x);
                }
            });
            for (MatOfPoint contour : contours)
            {
                rects.add(Imgproc.fitEllipse(new MatOfPoint2f(contour)));
            }

            targetPairs.clear();
            RotatedRect lastLefty = null;
            for (RotatedRect rect : rects)
            {
                //A "lefty" points at quadrant I or III
                //Otherwise, it's a "righty"
                if (Math.tan(Math.toRadians(rect.angle)) > 0.0)
                {
                    lastLefty = rect;
                }
                else
                {
                    if (lastLefty != null)
                    {
                        //A lefty followed by a righty forms a pair.
                        targetPairs.add(new TargetPair(lastLefty, rect));
                    }
                }
            }

            outdatedPairs = false;
        }
        return targetPairs;
    }
    /**
     * @return The angle in degrees of the pair's center relative to the robot's current orientation.
     */
    public double GetAngleToTargetPair(TargetPair pair)
    {
        //Avg. horizontal position of the pair of vision targets should be halfway between them
        double offset = pair.center.x - WIDTH / 2; //Now it's -160 to 160, with 0 being the center.
        return Math.toDegrees(Math.atan(offset / FOCAL_LEN));
    }
}