package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;
import org.opencv.aruco.Aruco;
import org.opencv.core.Mat;

import java.lang.reflect.Field;
import java.util.ArrayList;
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

import static java.lang.Math.cos;

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

    private static final double point_A_x = 11.21;
    private static final double point_A_z = 5;

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
            Log.d(TAG, "Scan AR Tag Start");;
            Mat arucoIDs = new Mat();
            List<Mat> arucoCorners = new ArrayList<>();
            Aruco.detectMarkers(
                    api.getMatNavCam(),
                    Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                    arucoCorners,
                    arucoIDs);
            Log.d(TAG, "Scan AR Tag Finish (Length:"+arucoIDs.size()+")");

            double xPixelOffset = 0;
            double yPixelOffset = 0;
            double xUnit = 0;
            double yUnit = 0;

            for (int i = 0; i < arucoCorners.size(); i++) {
                // Tag Data
                Mat tag = arucoCorners.get(i);
                double[] pixelPosLeftUp = tag.get(0, 0);
                double[] pixelPosLeftDown = tag.get(0, 3);
                double[] pixelPosRightUp = tag.get(0, 1);
                double[] pixelPosRightDown = tag.get(0, 2);
                xPixelOffset+=pixelPosRightDown[0]+pixelPosLeftUp[0]+pixelPosLeftDown[0]+pixelPosRightUp[0];
                yPixelOffset+=pixelPosRightDown[1]+pixelPosLeftUp[1]+pixelPosLeftDown[1]+pixelPosRightUp[1];
                xUnit += Math.abs((pixelPosRightUp[0]+pixelPosRightDown[0]-pixelPosLeftUp[0]-pixelPosLeftDown[0])/2/5);
                yUnit += Math.abs((pixelPosRightDown[1]-pixelPosLeftUp[1]+pixelPosLeftDown[1]-pixelPosRightUp[1])/2/5);
                Log.d(TAG,"AR Tag ID "+arucoIDs.get(i,0)[0]+" Unit:{x="+xUnit+" y="+yUnit+"}");
            }
            xPixelOffset/=16;
            yPixelOffset/=16;
            xUnit/=4;
            yUnit/=4;

            double centerX = 640;
            double centerY = 480;

            xPixelOffset-=centerX;
            yPixelOffset-=centerY;

            double dx = xPixelOffset/xUnit;
            double dy = yPixelOffset/yUnit;

            dx-=4.22;
            dy-=8.26;

            Log.d(TAG, "Target dX:" + dx + " dY:" + dy);

            double robot_to_wall = 134.83;

            double r = Math.sqrt(dx*dx + dy*dy +robot_to_wall*robot_to_wall);

            double roll = Math.atan2(dx, robot_to_wall) - 0.5 * Math.PI;
            double pitch = 0;
            double yaw = -Math.asin(dy/r);

            Log.d(TAG, "Yaw:" + yaw + " Pitch:" + pitch + " Roll:" + roll);
            Quaternion q = euler_to_quaternion(roll, pitch, yaw);
            Log.d(TAG, "Target Quaternion: " + q.toString());

            Point p = new Point(a_prime.getX()-0.0572/ cos(roll+0.5*Math.PI),a_prime.getY(),a_prime.getZ());
            //Point p = new Point(a_prime.getX(),a_prime.getY(),a_prime.getZ());
            Log.d(TAG,"Shift Pos: "+p.toString());
            Result result = api.moveTo(p, q, ENABLE_PRINT_ROBOT_LOCATION);
            Quaternion resultQu = api.getTrustedRobotKinematics().getOrientation();
            int loopCounter = 0;
            while ((!result.hasSucceeded() || Math.abs(resultQu.getX()) - q.getX() >= 0.01 || Math.abs(resultQu.getY() - q.getY()) >= 0.01 || Math.abs(resultQu.getZ() - q.getZ()) >= 0.01)
                    && loopCounter < 10) {
                Log.d(TAG, "Retry to rotation");
                result = api.moveTo(p, q, ENABLE_PRINT_ROBOT_LOCATION);
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
        moveToPoint(new Point(point_A_x, -9.8, point_A_z));
        moveToPoint(new Point(point_A_x, -10, point_A_z));
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

    public void moveToPointAPrime(Point a_prime, int KOZ_Pattern) {
        double prevX = point_A_x;
        double prevZ = point_A_z;
        double x = a_prime.getX();
        double y = a_prime.getY();
        double z = a_prime.getZ();
        switch (KOZ_Pattern) {
            case 1:
                if(x<=prevX){
                    if(z<prevZ){
                        if(x+0.22>prevX){
                            moveToPoint(new Point(x+0.25,y,z+0.5));
                        }
                        moveToPoint(new Point(x+0.25,y,z));
                    }
                }else{
                    if(x-0.46>prevX){
                        moveToPoint(new Point(x-0.5,y,z-0.25));
                    }
                    moveToPoint(new Point(x,y,z-0.25));
                }
                break;
            case 2:
                if(z<prevZ){
                    if (z+0.38<prevZ){
                        moveToPoint(new Point(x-0.25,y,z+0.5));
                    }
                    if (x<prevX){
                        moveToPoint(new Point(prevX,y,z));
                    }else{
                        moveToPoint(new Point(x-0.25,y,z));
                    }
                }
                break;
            case 3:
                if(x>=prevX){
                    if(z<prevZ){
                        if(x-0.22<prevX){
                            moveToPoint(new Point(x-0.25,y,z+0.5));
                        }
                        moveToPoint(new Point(x-0.25,y,z));
                    }
                }else{
                    if(x+0.46<prevX){
                        moveToPoint(new Point(x+0.5,y,z-0.25));
                    }
                    moveToPoint(new Point(x,y,z-0.25));
                }
                break;
            case 4:
                if(x<prevX){
                    if (x+0.38<prevX){
                        moveToPoint(new Point(x+0.5,y,z-0.25));
                    }
                    if (z<prevZ){
                        moveToPoint(new Point(x,y,prevZ));
                    }else{
                        moveToPoint(new Point(x,y,z-0.25));
                    }
                }
                break;
            case 5:
                if(x>=prevX){
                    if(z>prevZ){
                        if(x-0.22<prevX){
                            moveToPoint(new Point(x-0.25,y,z-0.5));
                        }
                        moveToPoint(new Point(x-0.25,y,z));
                    }
                }else{
                    if(z-0.38>prevZ){
                        moveToPoint(new Point(x-0.25,y,z-0.5));
                        moveToPoint(new Point(x-0.25,y,z));
                    }else{
                        moveToPoint(new Point(x+0.5,y,z+0.25));
                        moveToPoint(new Point(x,y,z+0.25));
                    }
                }
                break;
            case 6:
                if(z>prevZ){
                    if (z-0.38>prevZ){
                        moveToPoint(new Point(x-0.25,y,z-0.5));
                    }
                    if (x<prevX){
                        moveToPoint(new Point(prevX,y,z));
                    }else{
                        moveToPoint(new Point(x-0.25,y,z));
                    }
                }
                break;
            case 7:
                if(x<=prevX){
                    if(z>prevZ){
                        if(x+0.22>prevX){
                            moveToPoint(new Point(x+0.25,y,z-0.5));
                        }
                        moveToPoint(new Point(x+0.25,y,z));
                    }
                }else{
                    if(z-0.38>prevZ){
                        moveToPoint(new Point(x+0.25,y,z-0.5));
                        moveToPoint(new Point(x+0.25,y,z));
                    }else{
                        moveToPoint(new Point(x-0.5,y,z+0.25));
                        moveToPoint(new Point(x,y,z+0.25));
                    }
                }
                break;
            case 8:
                if(x>prevX){
                    if (x-0.38>prevX){
                        moveToPoint(new Point(x-0.5,y,z-0.25));
                    }
                    if (z<prevZ){
                        moveToPoint(new Point(x,y,prevZ));
                    }else{
                        moveToPoint(new Point(x,y,z-0.25));
                    }
                }
                break;
            default:
                break;
        }
        moveToPoint(a_prime);
    }

    public void moveToPointB(Point aprimeRef, int KOZ_Pattern) {
        double x = aprimeRef.getX();
        //double y = aprimeRef.getY();
        double z = aprimeRef.getZ();
        switch (KOZ_Pattern) {
            case 1:
                moveToPoint(new Point(x, -9, z - 0.25));
                moveToPoint(new Point(10.6, -9, z - 0.25));
                break;
            case 2:
                moveToPoint(new Point(10.6, -9, z));
                break;
            case 3:
                moveToPoint(new Point(10.6, -9, z));
                break;
            case 4:
                moveToPoint(new Point(10.6, -9, z));
                break;
            case 5:
                moveToPoint(new Point(10.6, -9, z));
                break;
            case 6:
                moveToPoint(new Point(10.6, -9, z));
                break;
            case 7:
                moveToPoint(new Point(x+0.25,-9,z));
                moveToPoint(new Point(x+0.25, -9, z - 0.5));
                moveToPoint(new Point(10.6, -9, z - 0.5));
                break;
            case 8:
                moveToPoint(new Point(x, -9, z - 0.25));
                moveToPoint(new Point(10.6, -9, z - 0.25));
                break;
            default:
                break;
        }
        Log.d(TAG, "B Point Node 1");
        moveToPoint(new Point(10.6, -8.0, 4.5));
        Log.d(TAG, "B Point Node 2");
        moveToPoint(new Point(10.6, -8.0, 4.5));
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
        double qx = Math.sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + cos(roll / 2) * Math.sin(pitch / 2) * Math.sin(yaw / 2);
        double qy = cos(roll / 2) * Math.sin(pitch / 2) * cos(yaw / 2) - Math.sin(roll / 2) * cos(pitch / 2) * Math.sin(yaw / 2);
        double qz = cos(roll / 2) * cos(pitch / 2) * Math.sin(yaw / 2) + Math.sin(roll / 2) * Math.sin(pitch / 2) * cos(yaw / 2);
        double qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - Math.sin(roll / 2) * Math.sin(pitch / 2) * Math.sin(yaw / 2);

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

