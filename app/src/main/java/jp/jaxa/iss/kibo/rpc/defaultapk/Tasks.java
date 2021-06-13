package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
//import android.os.AsyncTask;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

//import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Map;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;
//import jp.jaxa.iss.kibo.rpc.defaultapk.pathfinder.Node;
//import jp.jaxa.iss.kibo.rpc.defaultapk.pathfinder.PathFinder;

class Tasks {
    private static final String TAG=YourService.TAG;

    // Task to scan QR Code
    static class ScanTask{
        String execute(Bitmap... bitmaps) {
            Bitmap bin = bitmaps[0];
            int width = bin.getWidth();
            int height = bin.getHeight();
            int[] intArray = new int[width * height];
            Log.d(TAG, "[QR] getPixels");
            bin.getPixels(intArray, 0, width, 0, 0, width, height);
            bin.recycle();
            Log.d(TAG, "[QR] Convert to RGB");
            LuminanceSource LS = new RGBLuminanceSource(width, height, intArray);
            Log.d(TAG, "[QR] Convert to BinaryBitmap");
            BinaryBitmap BBM = new BinaryBitmap(new HybridBinarizer(LS));
            Map<DecodeHintType, Object> hints = new EnumMap<>(DecodeHintType.class);
            hints.put(DecodeHintType.CHARACTER_SET, "utf-8");
            //hints.put(DecodeHintType.POSSIBLE_FORMATS, BarcodeFormat.QR_CODE);
            try {
                Log.d(TAG,"[QR] Decode BinaryBitmap");
                com.google.zxing.Result R = new QRCodeReader().decode(BBM,hints);
                return R.getText();
            } catch (NotFoundException e) {
                e.printStackTrace();
                return "";
            } catch (FormatException e) {
                e.printStackTrace();
                return "";
            } catch (ChecksumException e) {
                e.printStackTrace();
                return "";
            }
        }
    }

    static class MoveTaskParameters{
        KiboRpcApi api;
        Point point;
        Quaternion quaternion;
        int maxRetry;
        boolean printRobotLocation;
        MoveTaskParameters(KiboRpcApi api, Point point, Quaternion quaternion, int maxRetry, boolean printRobotLocation){
            this.api=api;
            this.point=point;
            this.quaternion=quaternion;
            this.printRobotLocation=printRobotLocation;
            this.maxRetry=maxRetry;
        }
    }

    static class MoveTask{
        void execute(MoveTaskParameters... moveTaskParameters) {
            KiboRpcApi api=moveTaskParameters[0].api;
            Result result = api.moveTo(moveTaskParameters[0].point, moveTaskParameters[0].quaternion, moveTaskParameters[0].printRobotLocation);
            int loopCounter = 0;
            while (!result.hasSucceeded() && loopCounter < moveTaskParameters[0].maxRetry) {
                result = api.moveTo(moveTaskParameters[0].point, moveTaskParameters[0].quaternion, moveTaskParameters[0].printRobotLocation);
                ++loopCounter;
            }
        }
    }




}
