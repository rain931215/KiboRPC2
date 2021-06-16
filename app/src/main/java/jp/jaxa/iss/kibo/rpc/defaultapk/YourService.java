package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;
import org.opencv.aruco.Aruco;
import org.opencv.core.Mat;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.android.gs.MessageType;
import gov.nasa.arc.astrobee.android.gs.StartGuestScienceService;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.defaultapk.Tasks.MoveTask;
import jp.jaxa.iss.kibo.rpc.defaultapk.Tasks.MoveTaskParameters;
import jp.jaxa.iss.kibo.rpc.defaultapk.Tasks.ScanTask;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    public static final String TAG = "KiboAPP";
    private static final boolean ENABLE_PRINT_ROBOT_LOCATION = true;
    private static final int LOOP_MAX = 10;
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
        // Start Mission
        api.startMission();

        // Move To A Point
        Log.d(TAG, "Move To A Point Start");
        moveToPointA();
        Log.d(TAG, "Move To A Point Finish");

        // Scan QR Code
        Log.d(TAG, "Qr Code Scan Start");
        StringBuilder qrStrBuilder = new StringBuilder();
        int retry_MAX = 10;
        int time = 0;
        while (qrStrBuilder.length() == 0 && time < retry_MAX) {
            time++;
            moveToPoint(new Point(11.21, -10, 5));
            qrStrBuilder.append(scanQrCode());
        }
        boolean decodeSucceeded = false;
        int koz_pattern = 0;
        Point a_prime = null;
        if (qrStrBuilder.length() > 0) {
            String qrStr = qrStrBuilder.toString();
            Log.d(TAG, "QR Code Data: " + qrStr);
            // Send the discovered qr code's data to judge server
            api.sendDiscoveredQR(qrStr);
            try {
                // Deserialize the qr code's data
                JSONObject qrJson = new JSONObject(qrStr);
                koz_pattern = qrJson.getInt("p");
                a_prime = new Point(qrJson.getDouble("x"), qrJson.getDouble("y"), qrJson.getDouble("z"));
                decodeSucceeded = true;
            } catch (JSONException e) {
                e.printStackTrace();
            }
        }
        Log.d(TAG, "Qr Code Scan Finish");


        if (decodeSucceeded) {
            // Move To A' Point
            Log.d(TAG, "Move To A' Start");
            moveToPointAPrime(a_prime, koz_pattern);
            Log.d(TAG, "Move To A' Finish");

            // Scan AR Tag
            Log.d(TAG, "Scan AR Tag Start");
            //displayMsg(api,"Start Scan AR Tag");
            Mat arucoIDs = new Mat();
            List<Mat> arucoCorners = new ArrayList<>();
            Aruco.detectMarkers(
                    api.getMatNavCam(),
                    Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                    arucoCorners,
                    arucoIDs);
            Log.d(TAG, "Scan AR Tag Finish (Length:"+arucoIDs.size()+")");

            // Array is {leftRight's pixel/cm, upDown's pixel/cm}
            double[] leftUpUnit = {0,0}, leftDownUnit = {0,0}, rightUpUnit = {0,0}, rightDownUnit = {0,0};
            // Array is {x pixel in camera, y pixel in camera}
            double[] leftUpPixel = {0,0}, leftDownPixel = {0,0}, rightUpPixel = {0,0}, rightDownPixel = {0,0};

            for (int i = 0; i < arucoIDs.size().height; i++) {
                // Tag ID LeftUp=2 LeftDown=3 RightUp=1 RightDown=4
                int id = (int)arucoIDs.get(i,0)[0];
                // Tag Data
                Mat tag = arucoCorners.get(i);
                // n is pixel/cm
                double unitUpDown=0, unitLeftRight=0;
                double[] pixelPosLeftUp = tag.get(0, 0);
                double[] pixelPosLeftDown = tag.get(0, 3);
                double[] pixelPosRightUp = tag.get(0, 1);
                double[] pixelPosRightDown = tag.get(0, 2);
                Log.d(TAG,"AR Tag ID "+id+" Pixels:{"+ Arrays.toString(pixelPosLeftUp) +","+ Arrays.toString(pixelPosLeftDown) +","+ Arrays.toString(pixelPosRightUp) +","+ Arrays.toString(pixelPosRightDown) +"}");
                unitUpDown = (Math.abs(pixelPosRightDown[1]-pixelPosLeftUp[1])+Math.abs(pixelPosLeftDown[1]-pixelPosRightUp[1]))/2/5;
                unitLeftRight = (Math.abs(pixelPosRightDown[0]-pixelPosLeftUp[0])+Math.abs(pixelPosLeftDown[0]-pixelPosRightUp[0]))/2/5;
                switch (id){
                    case 1:
                        rightUpUnit=new double[]{unitLeftRight,unitUpDown};
                        rightUpPixel = new double[]{(pixelPosRightDown[0]+pixelPosLeftUp[0]+pixelPosLeftDown[0]+pixelPosRightUp[0])/4,(pixelPosRightDown[1]+pixelPosLeftUp[1]+pixelPosLeftDown[1]+pixelPosRightUp[1])/4};
                        break;
                    case 2:
                        leftUpUnit=new double[]{unitLeftRight,unitUpDown};
                        leftUpPixel = new double[]{(pixelPosRightDown[0]+pixelPosLeftUp[0]+pixelPosLeftDown[0]+pixelPosRightUp[0])/4,(pixelPosRightDown[1]+pixelPosLeftUp[1]+pixelPosLeftDown[1]+pixelPosRightUp[1])/4};
                        break;
                    case 3:
                        leftDownUnit=new double[]{unitLeftRight,unitUpDown};
                        leftDownPixel = new double[]{(pixelPosRightDown[0]+pixelPosLeftUp[0]+pixelPosLeftDown[0]+pixelPosRightUp[0])/4,(pixelPosRightDown[1]+pixelPosLeftUp[1]+pixelPosLeftDown[1]+pixelPosRightUp[1])/4};
                        break;
                    case 4:
                        rightDownUnit=new double[]{unitLeftRight,unitUpDown};
                        rightDownPixel = new double[]{(pixelPosRightDown[0]+pixelPosLeftUp[0]+pixelPosLeftDown[0]+pixelPosRightUp[0])/4,(pixelPosRightDown[1]+pixelPosLeftUp[1]+pixelPosLeftDown[1]+pixelPosRightUp[1])/4};
                        break;
                    default:
                        break;
                }
                Log.d(TAG,"AR Tag ID "+id+" Pixels:"+tag.toString()+" Unit: {"+unitLeftRight+","+unitUpDown+"}");
            }

            double averageUnitX = (rightDownUnit[0]+leftUpUnit[0]+leftDownUnit[0]+rightUpUnit[0])/2;
            double averageUnitY = (rightDownUnit[1]+leftUpUnit[1]+leftDownUnit[1]+rightUpUnit[1])/2;

            double centerX = 640;
            double centerY = 480;

            double dx = (leftUpPixel[0]+leftDownPixel[0]+rightUpPixel[0]+rightDownPixel[0]-4*centerX)/averageUnitX;
            double dy = (leftUpPixel[1]+leftDownPixel[1]+rightUpPixel[1]+rightDownPixel[1]-4*centerY)/averageUnitY;

            Log.d(TAG,"Pixels: {"+ Arrays.toString(leftUpPixel)+","+Arrays.toString(rightDownPixel)+","+Arrays.toString(rightUpPixel) +","+Arrays.toString(leftDownPixel)+"}");
            Log.d(TAG,"Average Unix x: "+averageUnitX+" y: "+averageUnitY);
            Log.d(TAG, "Target dX:" + dx + " dY:" + dy);

            double robot_to_wall = 134.83;

            double roll = Math.atan2(dx, robot_to_wall) - 0.5 * Math.PI;
            double pitch = 0;
            double yaw = -Math.atan2(dy, robot_to_wall);

            Log.d(TAG, "Yaw:" + yaw + " Pitch:" + pitch + " Roll:" + roll);
            Quaternion q = euler_to_quaternion(roll, pitch, yaw);
            Log.d(TAG, "Target Quaternion: " + q.toString());

            Point p = new Point(a_prime.getX()-0.0572/Math.cos(roll+0.5*Math.PI),-9.8,a_prime.getZ()+0.1111/Math.cos(yaw));
            Result result = api.moveTo(a_prime, q, ENABLE_PRINT_ROBOT_LOCATION);
            Quaternion resultQu = api.getTrustedRobotKinematics().getOrientation();
            int loopCounter = 0;
            while ((!result.hasSucceeded() || resultQu.getX() - q.getX() >= 0.01 || resultQu.getY() - q.getY() >= 0.01 || resultQu.getZ() - q.getZ() >= 0.01)
                    && loopCounter < 10) {
                Log.d(TAG, "Retry to rotation");
                result = api.moveTo(a_prime, q, ENABLE_PRINT_ROBOT_LOCATION);
                resultQu = api.getTrustedRobotKinematics().getOrientation();
                ++loopCounter;
            }

            Log.d(TAG, "Rotation Finish : " + api.getTrustedRobotKinematics().getOrientation().toString());
            Log.d(TAG, "Turn on laser");
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            Log.d(TAG, "Turn off laser");

            // Move to Point B Point
            Log.d(TAG, "Move To B Start");
            moveToPointB(a_prime, koz_pattern);
            Log.d(TAG, "Move To B Finish");
        }

        // Mission Finish
        Log.d(TAG, "Report Mission Completion");
        api.reportMissionCompletion();
    }

    public void moveToPointA() {
        moveToPoint(new Point(11.21, -9.8, 5));
        moveToPoint(new Point(11.21, -10, 5));
    }

    public String scanQrCode() {
        String qrStr;
        Bitmap navBitmap = api.getBitmapNavCam(); // x y 1280*960
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
        switch (KOZ_Pattern) {
            case 1:
                moveToPoint(new Point(x, y, z - 0.31));
                moved = true;
                break;
            case 2:
                moved = true;
                break;
            case 3:
                moveToPoint(new Point(x, y, z - 0.31));
                moved = true;
                break;
            case 4:
                moveToPoint(new Point(x, y, z - 0.31));
                moved = true;
                break;
            case 5:  // Works
                moveToPoint(new Point(x - 0.25, y, z - 0.485));
                moveToPoint(new Point(x - 0.25, y, z));
                moved = true;
                break;
            case 6:
                // TODO Need Smart Detect
                moveToPoint(new Point(x - 0.25, y, z - 0.485));
                moveToPoint(new Point(x - 0.25, y, z));
                moved = true;
                break;
            case 7:
                moveToPoint(new Point(x + 0.25, y, z - 0.485));
                moveToPoint(new Point(x + 0.25, y, z));
                moved = true;
                break;
            case 8:
                moveToPoint(new Point(x, y, z - 0.31));
                moved = true;
                break;
            default:
                break;
        }
        if (moved) {
            moveToPoint(point);
        }
    }

    public void moveToPointB(Point aprimeRef, int KOZ_Pattern) {
        // TODO Need Modify

        /*
         * This version is only for test purpose
         * Not for Judge
         */

        boolean moved = false;
        double x = aprimeRef.getX();
        //double y = aprimeRef.getY();
        double y = -9;
        double z = aprimeRef.getZ();
        switch (KOZ_Pattern) {
            case 1:
                moveToPoint(new Point(x, y, z - 0.31 <= 4.31 ? 4.31 : z - 0.31));
                moveToPoint(new Point(10.5, y, z - 0.31));
                moved = true;
                break;
            case 2:
                moveToPoint(new Point(10.5, y, z));
                moved = true;
                break;
            case 3:
                moveToPoint(new Point(10.5, y, z));
                moved = true;
                break;
            case 4:
                moveToPoint(new Point(10.5, y, z));
                moved = true;
                break;
            case 5:
                moveToPoint(new Point(10.5, y, z));
                moved = true;
                break;
            case 6:
                moveToPoint(new Point(10.5, y, z));
                moved = true;
                break;
            case 7:
                Log.d(TAG, "Pattern 7 Move to node 1");
                moveToPoint(new Point(x, y, 5.54));
                Log.d(TAG, "Pattern 7 Move to node 2");
                moveToPoint(new Point(10.5, y, 5.54));
                moved = true;
                break;
            case 8:
                moveToPoint(new Point(x, y, 5.54));
                moveToPoint(new Point(10.5, y, 5.54));
                moved = true;
                break;
            default:
                break;
        }
        if (moved) {
            Log.d(TAG, "B Point Node 1");
            moveToPoint(new Point(10.5, -8.0, 4.5));
            Log.d(TAG, "B Point Node 2");
            moveToPoint(new Point(10.6, -8.0, 4.5));
        }
    }

    public void moveToPoint(Point point) {
        if (point.getX() <= minX || point.getX() >= maxX) {
            Log.d(TAG, "[Move] Point's x is out of KIZ: " + point.getX());
            point = new Point(point.getX() <= minX ? minX : maxX, point.getY(), point.getZ());
        }
        if (point.getY() <= minY || point.getY() >= maxY) {
            Log.d(TAG, "[Move] Point's y is out of KIZ: " + point.getY());
            point = new Point(point.getX(), point.getY() <= minY ? minY : maxY, point.getZ());
        }
        if (point.getZ() <= minZ || point.getZ() >= maxZ) {
            Log.d(TAG, "[Move] Point's z is out of KIZ: " + point.getZ());
            point = new Point(point.getX(), point.getY(), point.getZ() <= minZ ? minZ : maxZ);
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

    static Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
        double qx = Math.sin(roll / 2) * Math.cos(pitch / 2) * Math.cos(yaw / 2) + Math.cos(roll / 2) * Math.sin(pitch / 2) * Math.sin(yaw / 2);
        double qy = Math.cos(roll / 2) * Math.sin(pitch / 2) * Math.cos(yaw / 2) - Math.sin(roll / 2) * Math.cos(pitch / 2) * Math.sin(yaw / 2);
        double qz = Math.cos(roll / 2) * Math.cos(pitch / 2) * Math.sin(yaw / 2) + Math.sin(roll / 2) * Math.sin(pitch / 2) * Math.cos(yaw / 2);
        double qw = Math.cos(roll / 2) * Math.cos(pitch / 2) * Math.cos(yaw / 2) - Math.sin(roll / 2) * Math.sin(pitch / 2) * Math.sin(yaw / 2);

        return new Quaternion((float) qz, (float) qy, (float) qx, (float) qw);
    }

    static void displayMsg(KiboRpcApi api,String msg){
        try {
            Field f = api.getClass().getDeclaredField("gsService");
            f.setAccessible(true);
            StartGuestScienceService gsService = (StartGuestScienceService) f.get(api);
            JSONObject data = new JSONObject();
            data.put("status", msg);
            gsService.sendData(MessageType.JSON, "data", data.toString());
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }
}

