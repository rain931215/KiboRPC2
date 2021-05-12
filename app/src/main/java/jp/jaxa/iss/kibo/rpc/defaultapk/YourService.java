package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;

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
    private static final boolean ENABLE_PRINT_ROBOT_LOCATION = false;
    private static final int LOOP_MAX = 3;
    private static final Quaternion generalQuaternion = new Quaternion(0, 0, -0.707f, 0.707f);

    @Override
    protected void runPlan1() {
        api.startMission();

        // Move To A Point
        Log.d(TAG, "Move To A Point Start");
        moveToPointA();
        Log.d(TAG, "Move To A Point Finish");

        String qrStr="";
        int retry_MAX = 5;
        int time=0;
        while (qrStr.isEmpty() && time<retry_MAX){
            time++;
            // Scan QR Code
            moveToPoint(new Point(11.21,-10,5));
            long currentMills = System.currentTimeMillis();
            qrStr = scanQrCode();
            Log.d(TAG, "Qr Code Scan " + (qrStr.isEmpty() ? "Failed" : "Succeeded") + " in " + (System.currentTimeMillis() - currentMills) + " ms");

        }
        boolean decodeSucceeded = false;
        int koz_pattern = 0;
        Point a_prime = null;
        if (!qrStr.isEmpty()) {
            Log.d(TAG, "Qr Code Data: " + qrStr);
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
        //moveToPoint(new Point(11.21,-10,5));
    }

    public String scanQrCode() {
        String qrStr;
        Log.d(TAG, "[QR] Qr Code Scan Started");
        Bitmap navBitmap = api.getBitmapNavCam();
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
            case 5:
                moveToPoint(new Point(x-0.25,y,z-0.38));
                moveToPoint(new Point(x-0.25,y,z));
                moved=true;
                break;
            case 6:
                // TODO Need Smart Detect
                moveToPoint(new Point(x-0.25,y,z-0.38));
                moveToPoint(new Point(x-0.25,y,z));
                moved=true;
                break;
            case 7:
                moveToPoint(new Point(x+0.25,y,z-0.38));
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
                moveToPoint(new Point(x,y,z-0.31));
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
                moveToPoint(new Point(x,y,z+0.31));
                moveToPoint(new Point(10.5,y,z+0.31));
                moved=true;
                break;
            case 8:
                moveToPoint(new Point(x,y,z+0.31));
                moveToPoint(new Point(10.5,y,z+0.31));
                moved=true;
                break;
            default:
                break;
        }
        if(moved){
            moveToPoint(new Point(10.5, -8.0, 4.5));
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
}

