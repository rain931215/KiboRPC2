package jp.jaxa.iss.kibo.rpc.defaultapk.pathfinder;

import android.util.Log;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PathFinder {
    private static final String TAG = "PathFinder";
    private static final int decAfterPoint = 1;
    private Vec3d startAbsouletePos, endRelativePos;
    private HashMap<String, Node> openNodeList, closeNodeList;
    private Node node;
    private List<Node> FList;
    private PointRule rule;

    public PathFinder(Vec3d startPoint, Vec3d goalPoint, PointRule rule) {
        startAbsouletePos = new Vec3d(startPoint.getX(),startPoint.getY(),startPoint.getZ());
        endRelativePos = new Vec3d(goalPoint.x - startPoint.x, goalPoint.y - startPoint.y, goalPoint.z - startPoint.z);
        openNodeList = new HashMap<>();
        closeNodeList = new HashMap<>();
        Vec3d pos = new Vec3d(0, 0, 0);
        Node firstNode = new Node(pos, null);
        openNodeList.put(pos.toString(), firstNode);
        FList = new ArrayList<>();
        FList.add(firstNode);
        this.rule = rule;
    }

    public ArrayList<Node> calculatePath() {
        Log.d(TAG,"Start Calculate Path");
        int count = 0;
        while (count < 20000) {
            count++;
            if (FList.size() < 1) {
                Log.d(TAG,"FList size < 1,Exiting");
                return null;
            }
            node = FList.get(0);
            //Log.d(TAG,"Remove FList index 0");
            FList.remove(0);
            if (node.getPos().equals(endRelativePos)) {
                Log.d(TAG,"OpenNodeList:"+openNodeList.size()+" CloseNodeList:"+closeNodeList.size());
                ArrayList<Node> list = new ArrayList<>();
                list.add(node);
                return node.returnNodes(list);
            }
            openNodeList.remove(node.getPos().toString());
            closeNodeList.put(node.getPos().toString(), node);
            Vec3d pos = node.getPos();
            for (double offSet = -0.1; offSet < 0.2; offSet += 0.2) {
                openNewNode(new Vec3d(pos.x + offSet, pos.y, pos.z));
                openNewNode(new Vec3d(pos.x, pos.y + offSet, pos.z));
                openNewNode(new Vec3d(pos.x, pos.y, pos.z + offSet));
            }
        }
        return null;
    }

    private void openNewNode(Vec3d pos) {
        if (nodeRule(pos)) { // 如果節點需要計算
            Node node2 = new Node(pos, node); // 產生新節點
            node2.setF(node2.getCost() + node2.getGuessCost(endRelativePos)); // 取得成本
            fListInsert(node2); // 插入節點
            openNodeList.put(pos.toString(), node2); // 加入節點
        }
    }

    private void clearNode(Node nodeToClear) {
        for (int i = 0; i < FList.size(); i++) {
            if (nodeToClear.getPos().equals(FList.get(i).getPos())) {
                FList.remove(i);
                return;
            }
        }
    }

    // 從小到大排序
    private void fListInsert(Node nodeToInsert) {
        for (int i = 0; i < FList.size(); i++) {
            Node v = FList.get(i);
            if (nodeToInsert.getF() == v.getF() || nodeToInsert.getF() < v.getF()) {
                FList.add(i, nodeToInsert); // TODO: Need Confirm
                return;
            }
        }
        FList.add(nodeToInsert);
    }

    // 節點判斷
    private boolean nodeRule(Vec3d pos) {
        pos = new Vec3d(round(pos.getX(),1),round(pos.getY(),1),round(pos.getZ(),1));
        // 判斷節點是否已經算過
        if (closeNodeList.get(pos.toString())!=null) return false;
        if (openNodeList.get(pos.toString())!=null) {
            Node v = openNodeList.get(pos.toString());
            if (v.getGuessCost(endRelativePos) < node.getGuessCost(endRelativePos)) {
                v.setLastNode(node);
                v.setCost();
                v.setF(v.getCost() + v.getGuessCost(endRelativePos));
                clearNode(v);
                fListInsert(v);
            }
            return false;
        }

        // 得出絕對座標
        double x = startAbsouletePos.x + pos.getX();
        double y = startAbsouletePos.y + pos.getY();
        double z = startAbsouletePos.z + pos.getZ();
        return rule.isPointAllow(new Vec3d(x, y, z));
    }


    public interface PointRule {
        boolean isPointAllow(Vec3d pointPos);
    }

    public static final class Vec3d {
        private double x, y, z;

        public Vec3d(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public double getX() {
            return round(x,decAfterPoint);
        }

        public double getY() {
            return round(y,decAfterPoint);
        }

        public double getZ() {
            return round(z,decAfterPoint);
        }

        @Override
        public boolean equals(Object obj) {
            if (!(obj instanceof Vec3d)) return false;
            Vec3d a = (Vec3d) obj;
            return a.getX() == getX() && a.getY() == getY() && a.getZ() == getZ();
        }

        @Override
        public String toString() {
            return getX() +"/"+getY()+"/"+getZ();
        }
    }
    private static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = BigDecimal.valueOf(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
