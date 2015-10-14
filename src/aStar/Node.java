package aStar;

import java.util.ArrayList;

/** Represents nodes in a map/board
 * 	Contains important information like terrain costs, f-cost, location etc.
 * 	Implements the Comparable interface for sorting on f-cost
 * 
 * @author Lars
 *
 */

public class Node implements Comparable<Node> {
        private ArrayList<Node> neighborList;
        private float costFromStart;
        private Node previousNode;
        private int x;
        private int y;
        private boolean isObstacle;
        private boolean isGoal;
        private boolean isStart;
        private float terrainCost;
        private float f_cost;
        private Character charType;
        
        Node(int x, int y) {
                neighborList = new ArrayList<Node>();
                this.x = x;
                this.y = y;
                this.costFromStart = Integer.MAX_VALUE;
                this.isObstacle = false;
                this.terrainCost = Integer.MAX_VALUE;
                this.f_cost = Integer.MAX_VALUE;
                this.isGoal = false;
        }
        
        Node (int x, int y, int costFromStart, boolean isObstical, boolean isGoal) {
                neighborList = new ArrayList<Node>();
                this.x = x;
                this.y = y;
                this.costFromStart = costFromStart;
                this.isObstacle = isObstical;
                this.isGoal = isGoal;
        }
        
        public ArrayList<Node> getNeighborList() {
                return neighborList;
        }

        public float getCostFromStart() {
                return costFromStart;
        }

        public void setCostFromStart(float f) {
                this.costFromStart = f;
        }

        public Node getPreviousNode() {
                return previousNode;
        }

        public void setPreviousNode(Node previousNode) {
                this.previousNode = previousNode;
        }

        public int getX() {
                return x;
        }

        public void setX(int x) {
                this.x = x;
        }

        public int getY() {
                return y;
        }

        public void setY(int y) {
                this.y = y;
        }
        
        public boolean isObstical() {
                return isObstacle;
        }

        public void setObstical(boolean isObstical) {
                this.isObstacle = isObstical;
        }

        public boolean isGoal() {
                return isGoal;
        }

        public void setGoal(boolean isGoal) {
                this.isGoal = isGoal;
        }
        
        public void setStart(boolean isStart) {
			this.isStart = isStart;
		}
        
        public boolean isStart(){
        	return this.isStart;
        }
        
        public boolean isObstacle(){
        	return this.isObstacle;
        }
        
        public void setTerrainCost(float terrainCost){
        	this.terrainCost = terrainCost;
        }
        
        public float getTerrainCost(){
        	return this.terrainCost;
        }
        
        public void setFCost(float f_cost){
        	this.f_cost = f_cost;
        }
        
        public float getFCost(){
        	return this.f_cost;
        }

        public boolean equals(Node node) {
                return (node.x == x) && (node.y == y);
        }
        
        public boolean isNeighborTo(Node node){
        	if(this.neighborList.contains(node)){
        		return true;
        	}
        	else return false;
        }
        
        public void setCharType(Character charType){
        	this.charType = charType;
        }
        
        public Character getCharType(){
        	return this.charType;
        }
        
        //compares the f_cost of two nodes
        public int compareTo(Node otherNode) {
                if (this.f_cost < otherNode.getFCost()) {
                        return -1;
                } else if (this.f_cost > otherNode.getFCost()) {
                        return 1;
                } else {
                        return 0;
                }
        }

}