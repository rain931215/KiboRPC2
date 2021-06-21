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

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.defaultapk.Tasks.MoveTask;
import jp.jaxa.iss.kibo.rpc.defaultapk.Tasks.MoveTaskParameters;
import jp.jaxa.iss.kibo.rpc.defaultapk.Tasks.ScanTask;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    public static final String TAG = "KiboAPP";
    public static final String TAG2 = "Laser Aiming";

    //the distance to target is 108.72cm while you are at the position of y=-9.95
    public static final double distance_center = 119.83;
    public static final double dx_laser = 0.0572;//m
    public static final double dy_laser = 0.1111;//m

    private static final double point_A_x = 11.21;
    private static final double point_A_z = 5;

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
        int retry_MAX = 10;
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
            //the laser center is (699.64,497.1)
            //the mass center is (669.0125,423.2125)

            double dx = (targetPixelX - 669.0125);
            double dy = (targetPixelY - 423.2125);
            Log.d(TAG2, "Target dX : " + dx + " dY : " + dy);

            final double pitch = 0;

            final double theta_x = Math.atan(Math.abs(dx)/ (distance_center * 6.875));
            final double theta_y = Math.atan(Math.abs(dy) / (distance_center * 6.875));


            final double displacement_x = Math.abs((dx_laser/Math.cos(theta_x)));
            final double displacement_y = Math.abs((dy_laser/Math.cos(theta_y)));

            //依據KOZ調整dis_x的關係
            double[] adjust = magnification(displacement_x, displacement_y, koz_pattern);

            final double dis_x = adjust[0];
            final double dis_y = adjust[1];
            final double multi_x = adjust[2];
            final double multi_y = adjust[3];


            //Log.d(TAG2,"displacement_x : "+displacement_x + " displacement_y : "+displacement_y);

            final double yaw = dx > 0 ? -0.5 * Math.PI + theta_x * multi_x : -0.5 * Math.PI - theta_x * multi_x;

            final double roll = dy > 0 ? -theta_y * multi_y : theta_y * multi_y;

            //還沒轉之前
            Log.d(TAG2, "Yaw in degree : " + yaw * (180 / Math.PI) + " Pitch : " + pitch * (180 / Math.PI) + " Roll : " + roll * (180 / Math.PI));

            Log.d(TAG2, "Yaw : " + yaw + " Pitch : " + pitch + " Roll : " + roll);


            final Point s = api.getTrustedRobotKinematics().getPosition();
            Log.d(TAG2, "The current position to aim the target");


            final Quaternion q = euler_to_quaternion(roll, pitch, yaw);

            Log.d(TAG2, "The Quaternion to rotate " + q.toString());

            //平移後的點
            final Point a_prime2 = new Point(a_prime.getX()+dis_x, a_prime.getY(), a_prime.getZ()+dis_y);

            //旋轉
            Result result = api.moveTo(a_prime2, q, ENABLE_PRINT_ROBOT_LOCATION);
            Quaternion resultQu = api.getTrustedRobotKinematics().getOrientation();
            int loopCounter = 0;
            while ((!result.hasSucceeded() || Math.abs(resultQu.getX() - q.getX()) >= 0.01 || Math.abs(resultQu.getY() - q.getY()) >= 0.01 || Math.abs(resultQu.getZ() - q.getZ()) >= 0.01)
                    && loopCounter < 10) {
                Log.d(TAG2, "Retry to rotation");
                result = api.moveTo(a_prime2, q, ENABLE_PRINT_ROBOT_LOCATION);
                resultQu = api.getTrustedRobotKinematics().getOrientation();
                ++loopCounter;
            }

            //平移失敗
            if (!result.hasSucceeded()){
                double[] adjust2 = magnification2(koz_pattern);
                final double pitch2 = 0;
                final double yaw2 = dx > 0 ? -0.5 * Math.PI + theta_x * adjust2[0] : -0.5 * Math.PI - theta_x * adjust2[0];
                final double roll2 = dy > 0 ? -theta_y * adjust2[1] : theta_y * adjust2[1];
                Quaternion q2 = euler_to_quaternion(roll2, pitch2, yaw2);
                api.moveTo(a_prime, q2, ENABLE_PRINT_ROBOT_LOCATION);
                Log.d(TAG2, "Only rotate function");
            }

            Log.d(TAG2, "Rotation Finish");


            final Point final_point = api.getTrustedRobotKinematics().getPosition();
            Log.d(TAG2,"The final position :"+final_point.toString());

            Log.d(TAG2, "Turn on laser");
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            Log.d(TAG2, "Turn off laser");

            //moveToPoint(a_prime);

            Log.d(TAG, "Move To B Start");
            moveToPointB(a_prime, koz_pattern);
            Log.d(TAG, "Move To B Finish");
        }
        else{
            moveToPoint(new Point(10.6, -9.0, 4.5));
            moveToPoint(new Point(10.6, -8.0, 4.5));
        }


        api.reportMissionCompletion();
        Log.d(TAG, "Report Mission Completion");



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


    public void moveToPointAPrime(Point a_prime, int KOZ_Pattern) {
        double prevX = point_A_x;
        double prevZ = point_A_z;
        double x = a_prime.getX();
        double y = a_prime.getY();
        double z = a_prime.getZ();
        switch (KOZ_Pattern) {
            case 1:
                moveToPoint(new Point(x, y, z - 0.31));
            case 2:
                if(z<prevZ){
                    Point currentPoint = api.getTrustedRobotKinematics().getPosition();
                    moveToPoint(new Point(currentPoint.getX(),y,z));
                }
                break;
            case 3:
                if(z<=prevZ){
                    Point currentPoint = api.getTrustedRobotKinematics().getPosition();
                    moveToPoint(new Point(currentPoint.getX(),y,z));
                }
                if (x+0.25<=prevX){
                    moveToPoint(new Point(x+0.5,y,z+0.25));
                    moveToPoint(new Point(x,y,z+0.25));
                }
                break;
            case 4:
                if(x+0.25<=prevX){
                    moveToPoint(new Point(x+0.5,y,z+0.25));
                }
                if(x<=prevX){
                    Point currentPoint = api.getTrustedRobotKinematics().getPosition();
                    moveToPoint(new Point(x,y,currentPoint.getZ()));
                }
                break;
            case 5:
                moveToPoint(new Point(x - 0.25, y, z - 0.485));
                moveToPoint(new Point(x - 0.25, y, z));
                break;
            //Todo
            case 6:
                moveToPoint(new Point(x - 0.25, y, z - 0.485));
                moveToPoint(new Point(x - 0.25, y, z));
                break;

            case 7:
                moveToPoint(new Point(x+0.25,y,z-0.5));
                moveToPoint(new Point(x+0.25,y,z));
                break;
            case 8:
                moveToPoint(new Point(x, y, z - 0.31));
                break;
            default:
                break;
        }
        moveToPoint(a_prime);
    }


    //all done
    public static double[] magnification(double dis_x, double dis_y, int KOZ_Pattern){
        double[] result = new double[4];
        switch (KOZ_Pattern)
        {
            case 1://finish
                result[0] = -dis_x*0.4;
                result[1] = -dis_y*0.95;
                result[2] = 2.5;
                result[3] = 1;
                break;
            case 2://finish
                result[0] = dis_x;
                result[1] = -dis_y*0.85;
                result[2] = 0.21;
                result[3] = 1;
                break;
            case 3://finish
                result[0] = dis_x;
                result[1] = -dis_y*0.85;
                result[2] = 0.21;
                result[3] = 1;
                break;
            case 4://finish
                result[0] = dis_x;
                result[1] = -dis_y*0.85;
                result[2] = 0.21;
                result[3] = 1;
                break;
            case 5://finish
                result[0] = -dis_x*0.3;
                result[1] = -dis_y*0.3;
                result[2] = 0.25;
                result[3] = 1;
                break;
            case 6://finish
                result[0] = -dis_x*0.3;
                result[1] = -dis_y*0.3;
                result[2] = 0.25;
                result[3] = 1;
                break;
            case 7://finish
                result[0] = 0;
                result[1] = -dis_y*0.33;
                result[2] = 0.78;
                result[3] = 1;
                break;
            case 8://tuning
                result[0] = -dis_x*0.4;
                result[1] = -dis_y*0.95;
                result[2] = 2.5;
                result[3] = 1;
                break;
            default:
                break;
        }
        Log.d(TAG2,"Multi_x : "+result[2]+" Multi_y : "+result[3]);
        return result;

    }

    //平移失敗function
    public static double[] magnification2(int KOZ_Pattern){
        double[] result = new double[2];
        switch (KOZ_Pattern)
        {
            case 1://finish
                result[0] = 2.2;
                result[1] = 0.75;
                break;
            case 2://tuning
                result[0] = 1.25;
                result[1] = 0.9;
                break;
            case 3://finish
                result[0] = 1.25;
                result[1] = 0.8;
                break;
            case 4://finish
                result[0] = 1.25;
                result[1] = 0.8;
                break;
            case 5://finish
                result[0] = 0.3;
                result[1] = 0.25;
                break;
            case 6://finish
                result[0] = 0.3;
                result[1] = 0.25;
                break;
            case 7://tuning
                result[0] = 1;
                result[1] = 1;
                break;
            case 8://finish
                result[0] = 2;
                result[1] = 0.85;
                break;
            default:
                break;
        }
        Log.d(TAG2,"Multi_x : "+result[0]+" Multi_y : "+result[1]);
        return result;

    }

    public void moveToPointB(Point aprimeRef, int KOZ_Pattern) {
        double x = aprimeRef.getX();
        double y = aprimeRef.getY();
        double z = aprimeRef.getZ();
        switch (KOZ_Pattern) {
            case 1:
                moveToPoint(new Point(x, y, 5));
                moveToPoint(new Point(10.6, -9, 5));
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
                moveToPoint(new Point(x+0.25,y,z));
                moveToPoint(new Point(x+0.25, y, 5));
                moveToPoint(new Point(10.6, -9, 4.5));
                break;
            case 8:
                moveToPoint(new Point(x, y , 5));
                moveToPoint(new Point(10.6, -9, 5));
                break;
            default:
                break;
        }
        Log.d(TAG, "B Point Node 1");
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


    public Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {

        final float sin_roll = (float) Math.sin(roll * 0.5f);
        final float cos_roll = (float) Math.cos(roll * 0.5f);

        final float sin_pitch = (float) Math.sin(pitch * 0.5f);
        final float cos_pitch = (float) Math.cos(pitch * 0.5f);

        final float sin_yaw = (float) Math.sin(yaw * 0.5f);
        final float cos_yaw = (float) Math.cos(yaw * 0.5f);

        float qx = (sin_roll * cos_pitch * cos_yaw) - (cos_roll * sin_pitch * sin_yaw)*(-1) ;
        float qy = (cos_roll * sin_pitch * cos_yaw) + (sin_roll * cos_pitch * sin_yaw)*(-1) ;
        float qz = (cos_roll * cos_pitch * sin_yaw) - (sin_roll * sin_pitch * cos_yaw)*(-1) ;
        float qw = (cos_roll * cos_pitch * cos_yaw) + (sin_roll * sin_pitch * sin_yaw)*(-1) ;

        return new Quaternion(qx, qy, qz, qw);

    }
    //final version
}


