package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;
import org.opencv.aruco.Aruco;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.defaultapk.pathfinder.PathFinder;
import jp.jaxa.iss.kibo.rpc.defaultapk.pathfinder.Node;

import jp.jaxa.iss.kibo.rpc.defaultapk.Tasks.*;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    public static final String TAG = "KiboAPP";
    private static final boolean ENABLE_PRINT_ROBOT_LOCATION = true;
    private static final int LOOP_MAX = 5;
    private static final Quaternion generalQuaternion = new Quaternion(0, 0, -0.707f, 0.707f);
    // 10.3 to 11.55
    private static final double minX = 10.35;
    private static final double maxX = 11.5;
    // -10.2 ~ -7.4
    private static final double minY = -10.2;
    private static final double maxY = -7.4;
    // 4.32 ~ 5.57
    private static final double minZ = 4.27;
    private static final double maxZ = 5.52;
    @Override
    protected void runPlan1() {
        api.startMission();

        Log.d(TAG,"Max Ram"+Runtime.getRuntime().maxMemory());
        // Move To A Point
        moveToPointA();
        Log.d(TAG, "Move To A Point Finish");

        StringBuilder qrStrBuilder=new StringBuilder();
        int retry_MAX = 5;
        int time=0;
        while (qrStrBuilder.length()==0 && time<retry_MAX){
            time++;
            // Scan QR Code
            moveToPoint(new Point(11.21,-10,5));
            qrStrBuilder.append(scanQrCode());
        }
        boolean decodeSucceeded = false;
        int koz_pattern = 0;
        Point a_prime = null;
        if (qrStrBuilder.length()>0) {
            String qrStr=qrStrBuilder.toString();
            Log.d(TAG, "QR CODE Data: " + qrStr);
            api.sendDiscoveredQR(qrStr); // Send the discovered qr code data to judge server
            try {
                JSONObject qrJson = new JSONObject(qrStr);
                koz_pattern = qrJson.getInt("p");
                a_prime = new Point(qrJson.getDouble("x"), qrJson.getDouble("y"), qrJson.getDouble("z"));
                decodeSucceeded = true;
            } catch (JSONException e) {
                e.printStackTrace();
            }
        }


        if (decodeSucceeded) {
            // Move To A'
            Log.d(TAG, "Move To A' Start");
            moveToPointAPrime(a_prime, koz_pattern);
            Log.d(TAG, "Move To A' Finish");

            Log.d(TAG,"Scan AR Tag Start");
            List<Mat> arucoCorners = new ArrayList<>();
            Mat arucoIDs = new Mat();
            Aruco.detectMarkers(
                    api.getMatNavCam(),
                    Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                    arucoCorners,
                    arucoIDs);
            Log.d(TAG,"Scan AR Tag Finish");
            Log.d(TAG,"AR Tag ID List Length: "+arucoIDs.size());
            int targetPixelX = 0;
            int targetPixelY = 0;
            for (int i=0;i<arucoIDs.size().height;i++) {
                Mat mat = arucoCorners.get(i); // 1 row 4 column(1 LeftUp 2 RightUp 3 RightDown 4 LeftDown)
                double[] pixelLeftUp = mat.get(0,0);
                double[] pixelLeftDown = mat.get(0,1);
                double[] pixelRightUP = mat.get(0,2);
                double[] pixelRightDown = mat.get(0,3);
                targetPixelX+=pixelLeftUp[0]+pixelLeftDown[0]+pixelRightUP[0]+pixelRightDown[0];
                targetPixelY+=pixelLeftUp[1]+pixelLeftDown[1]+pixelRightUP[1]+pixelRightDown[1];
                Log.d(TAG,"AR Tag ID "+ Arrays.toString(arucoIDs.get(i, 0)) +" Data: {"+ Arrays.toString(pixelLeftUp) +","+ Arrays.toString(pixelLeftDown) +","+ Arrays.toString(pixelRightUP) +","+ Arrays.toString(pixelRightDown)+"}");
            }
            targetPixelX/=16;
            targetPixelY/=16;
            Log.d(TAG,"Target X:"+targetPixelX+" Y:"+targetPixelY);
            // 6 pixel is 1 cm
            // Screen is H:960 W:1280 (Center: 480 640)
            // dx dy is cm
            double dx = (targetPixelX-640)/6.0;
            double dy = (targetPixelY-480)/6.0;
            Log.d(TAG,"Target dX:"+dx+" dY:"+dy);
            Point p = api.getTrustedRobotKinematics().getPosition();
            // unit is m
//            double yaw = 45-Math.toDegrees(Math.atan2(9.8,dx/100));
//            double pitch = 0;
//            double roll = Math.toDegrees(Math.atan2(9.8,dy/100))-90;
            double yaw = Math.atan(dx/100/9.8)-0.5*Math.PI*0.98;
            double pitch = 0;
            double roll = Math.atan(dy/100/9.8);

            Log.d(TAG,"Yaw:"+yaw+" Pitch:"+pitch+" Roll:"+roll);
            Quaternion q = euler_to_quaternion(yaw,pitch,roll);
            new MoveTask().execute(
                    new MoveTaskParameters(
                            api,
                            p,
                            q,
                            LOOP_MAX,
                            ENABLE_PRINT_ROBOT_LOCATION
                    )
            );

            Log.d(TAG,"Turn on laser");
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            Log.d(TAG,"Turn off laser");

            Log.d(TAG, "Move To B Start");
            moveToPointB(a_prime,koz_pattern);
            Log.d(TAG, "Move To B Finish");
        }


        Log.d(TAG, "Report Mission Completion");
        api.reportMissionCompletion();
    }

    public void moveToPointA() {
        moveToPoint(new Point(11.21,-9.8,5));
        moveToPoint(new Point(11.21,-10,5));
    }

    public String scanQrCode() {
        String qrStr;
        Log.d(TAG, "[QR] Qr Code Scan Started");
        Bitmap navBitmap = api.getBitmapNavCam();
        //Log.d(TAG,"Bitmap H:"+navBitmap.getHeight()+" W:"+navBitmap.getWidth());
        Bitmap crop = Bitmap.createBitmap(navBitmap, 605, 460, 320, 240);
        navBitmap.recycle();
        qrStr = new ScanTask().execute(crop);
        crop.recycle();
        return qrStr;
    }

    public void moveToPointAPrime(Point point, int KOZ_Pattern) {
        boolean moved = false;
        double x = point.getX();
        double y = point.getY();
        double z = point.getZ();
        switch (KOZ_Pattern){
            case 1:
                moveToPoint(new Point(x,y,z-0.31));
                moved=true;
                break;
            case 2:
                moved=true;
                break;
            case 3:
                moveToPoint(new Point(x,y,z-0.31));
                moved=true;
                break;
            case 4:
                moveToPoint(new Point(x,y,z-0.31));
                moved=true;
                break;
            case 5:  // Works
                moveToPoint(new Point(x-0.25,y,z-0.485));
                moveToPoint(new Point(x-0.25,y,z));
                moved=true;
                break;
            case 6:
                // TODO Need Smart Detect
                moveToPoint(new Point(x-0.25,y,z-0.485));
                moveToPoint(new Point(x-0.25,y,z));
                moved=true;
                break;
            case 7:
                moveToPoint(new Point(x+0.25,y,z-0.485));
                moveToPoint(new Point(x+0.25,y,z));
                moved=true;
                break;
            case 8:
                moveToPoint(new Point(x,y,z-0.31));
                moved=true;
                break;
            default:
                break;
        }
        if(moved){
            moveToPoint(point);
        }
    }

    public void moveToPointB(Point aprimeRef,int KOZ_Pattern) {
        // TODO Need Modify

        /*
        * This version is only for test purpose
        * Not for Judge
        */

        boolean moved = false;
        double x = aprimeRef.getX();
        double y = aprimeRef.getY();
        double z = aprimeRef.getZ();
        switch (KOZ_Pattern){
            case 1:
                moveToPoint(new Point(x,y,z-0.31<=4.31?4.31:z-0.31));
                moveToPoint(new Point(10.5,y,z-0.31));
                moved=true;
                break;
            case 2:
                moveToPoint(new Point(10.5,y,z));
                moved=true;
                break;
            case 3:
                moveToPoint(new Point(10.5,y,z));
                moved=true;
                break;
            case 4:
                moveToPoint(new Point(10.5,y,z));
                moved=true;
                break;
            case 5:
                moveToPoint(new Point(10.5,y,z));
                moved=true;
                break;
            case 6:
                moveToPoint(new Point(10.5,y,z));
                moved=true;
                break;
            case 7:
                Log.d(TAG,"Pattern 7 Move to node 1");
                moveToPoint(new Point(x,y,5.54));
                Log.d(TAG,"Pattern 7 Move to node 2");
                moveToPoint(new Point(10.5,y,5.54));
                moved=true;
                break;
            case 8:
                moveToPoint(new Point(x,y,5.54));
                moveToPoint(new Point(10.5,y,5.54));
                moved=true;
                break;
            default:
                break;
        }
        if(moved){
            Log.d(TAG,"B Point Node 1");
            moveToPoint(new Point(10.5, -8.0, 4.5));
            Log.d(TAG,"B Point Node 2");
            moveToPoint(new Point(10.6, -8.0, 4.5));
        }
    }

    public void moveToPointWithNavigation(Point goalPoint, Quaternion quaternion, boolean printRobotPosition) {
        Point p = api.getRobotKinematics().getPosition();
        PathFinder.Vec3d currentPos = new PathFinder.Vec3d(p.getX(), p.getY(), p.getZ());
        PathFinder finder1 = new PathFinder(currentPos, new PathFinder.Vec3d(goalPoint.getX(), goalPoint.getY(), goalPoint.getZ()), pointPos -> true);

        List<Node> successNodes = finder1.calculatePath();
//        PathFinder finder2 = new PathFinder(new PathFinder.Vec3d(goalPoint.getX(), goalPoint.getY(), goalPoint.getZ()), currentPos, this);
//        final List<List<Node>> successNodesList = new ArrayList<>();
//        final boolean[] reverse = new boolean[1];
//        Thread main = Thread.currentThread();
//        Thread[] threads = new Thread[2];
//        threads[0] = new Thread(() -> {
//            Log.d(TAG, "[PathFinder] PathFinder Thread A Started");
//            List<Node> nodes = finder1.calculatePath();
//            Log.d(TAG, "[PathFinder] PathFinder Thread A Calculated");
//            if (nodes != null && successNodesList.size() == 0) {
//                successNodesList.add(nodes);
//                reverse[0] = false;
//                Log.d(TAG, "[PathFinder] PathFinder Thread A Finished L:" + nodes.size());
//                synchronized (main) {
//                    main.notify();
//                }
//            }
//        });
//        threads[1] = new Thread(() -> {
//            Log.d(TAG, "[PathFinder] PathFinder Thread B Started");
//            List<Node> nodes = finder2.calculatePath();
//            Log.d(TAG, "[PathFinder] PathFinder Thread B Calculated");
//            if (nodes != null && successNodesList.size() == 0) {
//                successNodesList.add(nodes);
//                reverse[0] = true;
//                Log.d(TAG, "[PathFinder] PathFinder Thread B Finished L:" + nodes.size());
//                synchronized (main) {
//                    main.notify();
//                }
//            }
//        });
//        Log.d(TAG, "[PathFinder] Starting PathFinder's Thread");
//        threads[0].start();
//        //threads[1].start();
//        Log.d(TAG, "[PathFinder] Started PathFinder's Thread");
//        synchronized (main) {
//            try {
//                Log.d(TAG, "[PathFinder] Main Thread Entered To Wait State");
//                main.wait();
//                Log.d(TAG, "[PathFinder] Main Thread Recovered From Wait State");
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//
//        List<Node> successNodes = successNodesList.get(0);
        if (successNodes.size() > 0) {
            Log.d(TAG, "[PathFinder] Founded Path,Generating Full");
            successNodes = sortNodes(successNodes);
            for (int i = successNodes.size() - 1; i != 0; i--) {
                double dx, dy, dz;
                PathFinder.Vec3d nodePos = successNodes.get(i).getPos();
                //if (reverse[0]) {
                if (false) {
                    dx = goalPoint.getX() + nodePos.getX();
                    dy = goalPoint.getY() + nodePos.getY();
                    dz = goalPoint.getZ() + nodePos.getZ();
                } else {
                    dx = currentPos.getX() + nodePos.getX();
                    dy = currentPos.getY() + nodePos.getY();
                    dz = currentPos.getZ() + nodePos.getZ();
                }
                Log.d(TAG, "PathFinder Move To " + dx + " " + dy + " " + dz);
                api.moveTo(new Point(dx, dy, dz), quaternion, printRobotPosition);
            }
        }
    }

    public List<Node> sortNodes(List<Node> nodes) {
        return nodes;
    }

    public void moveToPoint(Point point){
        if(point.getX()<=minX || point.getX()>=maxX){
            Log.d(TAG,"[Move] Point's x is out of KIZ: "+point.getX());
            point = new Point(point.getX()<=minX?minX:maxX,point.getY(),point.getZ());
        }
        if(point.getY()<=minY||point.getY()>=maxY){
            Log.d(TAG,"[Move] Point's y is out of KIZ: "+point.getY());
            point = new Point(point.getX(),point.getY()<=minY?minY:maxY,point.getZ());
        }
        if(point.getZ()<=minZ||point.getZ()>=maxZ){
            Log.d(TAG,"[Move] Point's z is out of KIZ: "+point.getZ());
            point = new Point(point.getX(),point.getY(),point.getZ()<=minZ?minZ:maxZ);
        }
        new MoveTask().execute(
                new MoveTaskParameters(
                        api,
                        point,
                        generalQuaternion,
                        LOOP_MAX,
                        ENABLE_PRINT_ROBOT_LOCATION
                )
        );
    }
    Quaternion euler_to_quaternion(double yaw,double pitch,double roll){
        Double qx = Math.sin(roll/2) * Math.cos(pitch/2) * Math.cos(yaw/2) - Math.cos(roll/2) * Math.sin(pitch/2) * Math.sin(yaw/2);
        Double qy = Math.cos(roll/2) * Math.sin(pitch/2) * Math.cos(yaw/2) + Math.sin(roll/2) * Math.cos(pitch/2) * Math.sin(yaw/2);
        Double qz = Math.cos(roll/2) * Math.cos(pitch/2) * Math.sin(yaw/2) - Math.sin(roll/2) * Math.sin(pitch/2) * Math.cos(yaw/2);
        Double qw = Math.cos(roll/2) * Math.cos(pitch/2) * Math.cos(yaw/2) + Math.sin(roll/2) * Math.sin(pitch/2) * Math.sin(yaw/2);
        return new Quaternion(qx.floatValue(),qy.floatValue(),qz.floatValue(),qw.floatValue());
    }
}

