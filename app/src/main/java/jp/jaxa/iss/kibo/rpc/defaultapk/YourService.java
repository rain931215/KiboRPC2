package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
//import android.nfc.Tag;
import android.util.Log;

//import org.apache.commons.lang.ObjectUtils;
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
//import jp.jaxa.iss.kibo.rpc.defaultapk.pathfinder.PathFinder;
//import jp.jaxa.iss.kibo.rpc.defaultapk.pathfinder.Node;

import jp.jaxa.iss.kibo.rpc.defaultapk.Tasks.*;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    public static final String TAG = "KiboAPP";
    public static final String TAG2 = "Laser Aiming";

    //the distance to target is 108.72cm while you are at the position of y=-9.95
    public static final double distance_center = 119.83;
    public static final double dx_laser = 0.0572;
    public static final double dy_laser = 0.1111;

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

        Log.d(TAG, "Max Ram" + Runtime.getRuntime().maxMemory());
        // Move To A Point
        moveToPointA();
        Log.d(TAG, "Move To A Point Finish");

        StringBuilder qrStrBuilder = new StringBuilder();
        int retry_MAX = 5;
        int time = 0;
        while (qrStrBuilder.length() == 0 && time < retry_MAX) {
            time++;
            // Scan QR Code
            moveToPoint(new Point(11.21, -10, 5));
            qrStrBuilder.append(scanQrCode());
        }
        boolean decodeSucceeded = false;
        int koz_pattern = 0;
        Point a_prime = null;
        if (qrStrBuilder.length() > 0) {
            String qrStr = qrStrBuilder.toString();
            Log.d(TAG, "QR CODE Data: " + qrStr);
            api.sendDiscoveredQR(qrStr); // Send the discovered qr code data to judge server
            try {
                JSONObject qrJson = new JSONObject(qrStr);
                koz_pattern = qrJson.getInt("p");
                a_prime = new Point(qrJson.getDouble("x"), -9.95, qrJson.getDouble("z"));
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

            Log.d(TAG, "Scan AR Tag Start");
            List<Mat> arucoCorners = new ArrayList<>();

            //final Point p = api.getTrustedRobotKinematics().getPosition();
            Mat arucoIDs = new Mat();

            Aruco.detectMarkers(
                    api.getMatNavCam(),
                    Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250),
                    arucoCorners,
                    arucoIDs);

            Log.d(TAG, "Scan AR Tag Finish");
            Log.d(TAG, "AR Tag ID List Length: " + arucoIDs.size());

            double targetPixelX = 0;
            double targetPixelY = 0;

            for (int i = 0; i < arucoIDs.size().height; i++) {
                Mat mat = arucoCorners.get(i); // 1 row 4 column(1 LeftUp 2 RightUp 3 RightDown 4 LeftDown)
                double[] pixelLeftUp = mat.get(0, 0);
                double[] pixelLeftDown = mat.get(0, 1);
                double[] pixelRightUP = mat.get(0, 2);
                double[] pixelRightDown = mat.get(0, 3);
                targetPixelX += pixelLeftUp[0] + pixelLeftDown[0] + pixelRightUP[0] + pixelRightDown[0];
                targetPixelY += pixelLeftUp[1] + pixelLeftDown[1] + pixelRightUP[1] + pixelRightDown[1];
                Log.d(TAG, "AR Tag ID " + Arrays.toString(arucoIDs.get(i, 0)) + " Data: {" + Arrays.toString(pixelLeftUp) + "," + Arrays.toString(pixelLeftDown) + "," + Arrays.toString(pixelRightUP) + "," + Arrays.toString(pixelRightDown) + "}");
            }


            targetPixelX /= 16;
            targetPixelY /= 16;
            //targetPixelY = targetPixelY>0?targetPixelY+5:targetPixelY-5;

            Log.d(TAG2, "Target X:" + targetPixelX + " Y:" + targetPixelY);

            // 6.875 pixel is 1 cm
            // Screen is H:960 W:1280 Center:(640,480)
            // dx dy is cm
            //actually,the ture center is (699.64,497.1)

            double dx = (targetPixelX - 669.0125);
            double dy = (targetPixelY - 423.2125);
            Log.d(TAG2, "Target dX:" + dx + " dY:" + dy);

            final double pitch = 0;

            final double theta_x = Math.atan(Math.abs(dx)/ (distance_center * 6.875));
            final double theta_y = Math.atan(Math.abs(dy) / (distance_center * 6.875));

            //the unit is m
            final double displacement_x = Math.abs((dx_laser/Math.cos(theta_x)));
            final double displacement_y = Math.abs((dy_laser/Math.cos(theta_y)));

            final double yaw = dx > 0 ? -0.5 * Math.PI + theta_x : -0.5 * theta_x;

            final double roll = dy > 0 ? -theta_y : theta_y;

            //還沒轉之前
            Log.d(TAG2, "Yaw in degree:" + yaw * (180 / Math.PI) + " Pitch:" + pitch * (180 / Math.PI) + " Roll:" + roll * (180 / Math.PI));

            Log.d(TAG2, "Yaw:" + yaw + " Pitch:" + pitch + " Roll:" + roll);


            final Point s = api.getTrustedRobotKinematics().getPosition();
            Log.d(TAG2, "The current position to aim the target");


            final Quaternion q = euler_to_quaternion(roll, pitch, yaw, a_prime);

            Log.d(TAG2, "The Quaternion to rotate " + q.toString());

            api.moveTo(a_prime, q, ENABLE_PRINT_ROBOT_LOCATION);

//            new MoveTask().execute(
//                    new MoveTaskParameters(
//                            api,
//                            s,
//                            q,
//                            LOOP_MAX,
//                            ENABLE_PRINT_ROBOT_LOCATION
//                    )
//            );
            Log.d(TAG2, "Rotation Finish");

            moveToPoint(new Point(a_prime.getX()-displacement_x, a_prime.getY(), a_prime.getZ()+displacement_y));
            Log.d(TAG2,"Position fixed");

            Log.d(TAG2, "Turn on laser");
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            Log.d(TAG2, "Turn off laser");

            Log.d(TAG, "Move To B Start");
            moveToPointB(a_prime, koz_pattern);
            Log.d(TAG, "Move To B Finish");
        }


        Log.d(TAG, "Report Mission Completion");
        api.reportMissionCompletion();
    }


    public void moveToPointA() {
        moveToPoint(new Point(11.21, -9.8, 5));
        moveToPoint(new Point(11.21, -10, 5));
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
        double y = aprimeRef.getY();
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
                moveToPoint(new Point(x, y, z - 0.31 <= 4.31 ? 4.31 : z - 0.31));
                moveToPoint(new Point(10.5, y, z - 0.31));
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

    public Quaternion euler_to_quaternion(double roll, double pitch, double yaw, Point a_prime) {

        final float sin_roll = (float) Math.sin(roll * 0.5f);
        final float cos_roll = (float) Math.cos(roll * 0.5f);

        final float sin_pitch = (float) Math.sin(pitch * 0.5f);
        final float cos_pitch = (float) Math.cos(pitch * 0.5f);

        final float sin_yaw = (float) Math.sin(yaw * 0.5f);
        final float cos_yaw = (float) Math.cos(yaw * 0.5f);

        final int multi = a_prime.getX() > 0 ? -1 : 1;

        float qx = (sin_roll * cos_pitch * cos_yaw) - (cos_roll * sin_pitch * sin_yaw) * multi;
        float qy = (cos_roll * sin_pitch * cos_yaw) + (sin_roll * cos_pitch * sin_yaw) * multi;
        float qz = (cos_roll * cos_pitch * sin_yaw) - (sin_roll * sin_pitch * cos_yaw) * multi;
        float qw = (cos_roll * cos_pitch * cos_yaw) + (sin_roll * sin_pitch * sin_yaw) * multi;

        return new Quaternion(qx, qy, qz, qw);
    }
}

