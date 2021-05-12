package jp.jaxa.iss.kibo.rpc.defaultapk.pathfinder;

import java.util.ArrayList;

public class Node {
    private PathFinder.Vec3d pos;
    private Node lastNode;
    private double cost=0;
    private double f=0;
    Node(PathFinder.Vec3d pos, Node lastNode){
        this.pos=pos;
        this.lastNode=lastNode;

    }

    void setCost(){
        cost = lastNode==null?0:lastNode.getCost()+10;
    }

    double getCost() {
        return cost;
    }
    double getGuessCost(PathFinder.Vec3d goalPoint){
        return 350 * (Math.abs(goalPoint.getX()-pos.getX())+ Math.abs(goalPoint.getY()-pos.getY())+Math.abs(goalPoint.getZ()-pos.getZ()));
    }
    ArrayList<Node> returnNodes(ArrayList<Node> nodes){
        if (lastNode != null) {
            nodes.add(lastNode);
            nodes = lastNode.returnNodes(nodes);
        } else {
            return nodes;
        }
        return nodes;
    }

    public PathFinder.Vec3d getPos() {
        return pos;
    }

    void setLastNode(Node lastNode) {
        this.lastNode = lastNode;
    }

    void setF(double f) {
        this.f = f;
    }

    double getF() {
        return f;
    }
}
