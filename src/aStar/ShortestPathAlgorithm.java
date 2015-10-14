package aStar;

import java.util.ArrayList;
import java.util.Collections;

import aStar.heuristics.AStarHeuristic;
import aStar.utils.Logger;

/** This class i used for calculating the shortest path through a grid of Node objects
 * 	The class contains methods for calculating the shortest path, printing the path, and som other minor methods
 * 
 * @author Lars
 *
 */
public class ShortestPathAlgorithm {
        private AreaMap map;
        private AStarHeuristic heuristic;
        
        /**
         * closedList The list of Nodes not searched yet, sorted by their distance to the goal as guessed by our heuristic.
         */
        private ArrayList<Node> aStarClosedList;
        private SortedNodeList aStarOpenList;
        private ArrayList<Node> dijkstraClosedList;
        private SortedNodeList dijkstraOpenList;
        private ArrayList<Node> bfsClosedList;
        private SortedNodeList bfsOpenList;
        private ArrayList<Node> lastClosedList;
        private SortedNodeList lastOpenList;
        private Path aStarShortestPath;
        private Path dijkstraShortestPath;
        private Path bfsShortestPath;
        private static float aStarPathCost;
        private static float dijkstraPathCost;
        private static float bfsPathCost;
        Logger log = new Logger();
        private boolean finished;
        Node current;

        //Initializes variables
        ShortestPathAlgorithm(AreaMap map, AStarHeuristic heuristic) {
                this.map = map;
                this.heuristic = heuristic;
                aStarClosedList = new ArrayList<Node>();
                aStarOpenList = new SortedNodeList();
                dijkstraClosedList = new ArrayList<Node>();
                dijkstraOpenList = new SortedNodeList();
                bfsClosedList = new ArrayList<Node>();
                bfsOpenList = new SortedNodeList();
                lastClosedList = new ArrayList<Node>();
                lastOpenList = new SortedNodeList();
                finished = false;
                aStarPathCost = 0;
                dijkstraPathCost = 0;
                bfsPathCost = 0;
        }

        /** This is our implementation of the A-STAR algorithm
         * 	Based on the pseudocode on wikipedia
         * @return
         */
        public Path calcShortestPathUsingAStar() {
        		finished = false;
        		//reset values in nodes that were entered by previous algorithms
        		resetEnteredNodeValues(lastClosedList, lastOpenList);
                map.getStartNode().setCostFromStart(0); //setter avstand fra start til start lik 0
                aStarOpenList.add(map.getStartNode(), true);
                //while we haven't reached the goal yet
                while(!finished) {
                		//get the first Node from non-searched Node list, sorted by lowest distance from our goal as guessed by our heuristic
                		current = aStarOpenList.getFirst();
                		//move current Node to the closed (already searched) list
                		aStarOpenList.remove(current);
                    	//Initialize current's neighbors ?? or initialize when registering edges?? Or just drop registering edges??
                		map.registerNodeNeighbors(current);
                        aStarClosedList.add(current);
                        //go through all the current Nodes neighbors and calculate if one should be our next step
                        for(Node neighbor : current.getNeighborList()) {
                            	// check if the neighbor is the goal Node. If it is, we are done.
	                        	if(neighbor.isGoal()) {
	                        		neighbor.setPreviousNode(current);
	                        		finished = true;
	                        		break;
	                        	}
                                //continue if neighbor is an obstacle or it is contained in the closedList
                                if(neighbor.isObstacle() || aStarClosedList.contains(neighbor)) {
                                		continue;
                                }
                                // calculate g(n), the cost to get from start to neighbor node n with the current path
                                float neighborCostFromStart = (current.getCostFromStart() + map.getCostBetween(current, neighbor));
                                //add neighbor to the open list if it is not there or if the new path is shorter 
                                if(!aStarOpenList.contains(neighbor) || (neighborCostFromStart < neighbor.getCostFromStart())) {
                            			neighbor.setPreviousNode(current);
                                		neighbor.setCostFromStart(neighborCostFromStart);
                                		// calculate h(n), the heuristic which is the distance from the neighbor node to the goal
                                        float estimatedDistanceToGoal = heuristic.getEstimatedDistanceToGoal(neighbor.getX(), neighbor.getY(), map.getGoalLocationX(), map.getGoalLocationY());
                                        neighbor.setFCost(neighborCostFromStart + estimatedDistanceToGoal);
                                        if(!aStarOpenList.contains(neighbor))
                                        	aStarOpenList.add(neighbor, true);
                                } else continue;
                                }

                        }
                		lastOpenList = aStarOpenList;
                		lastClosedList = aStarClosedList;
                		return aStarShortestPath = reconstructPath(current, true, false, false);

        }
        
        /** This is our implementation of the DIJKSTRA algorithm
         * 	Only difference from A-STAR is that it does not include the heuristic
         * @return
         */
        public Path calcShortestPathUsingDijkstra() {
        	finished = false;
    		//reset values in nodes that were entered by previous algorithms
    		resetEnteredNodeValues(lastClosedList, lastOpenList);
            map.getStartNode().setCostFromStart(0); //setter avstand fra start til start lik 0
            dijkstraOpenList.add(map.getStartNode(), true);
            //while we haven't reached the goal yet
            while(!finished) {
            		//get the first Node from non-searched Node list, sorted by lowest distance from our goal as guessed by our heuristic
            		current = dijkstraOpenList.getFirst();
            		//move current Node to the closed (already searched) list
            		dijkstraOpenList.remove(current);
                	//Initialize current's neighbors ?? or initialize when registering edges?? Or just drop registering edges??
            		map.registerNodeNeighbors(current);
                    dijkstraClosedList.add(current);
                    //go through all the current Nodes neighbors and calculate if one should be our next step
                    for(Node neighbor : current.getNeighborList()) {
                        	// check if the neighbor is the goal Node. If it is, we are done.
                        	if(neighbor.isGoal()) {
                        		neighbor.setPreviousNode(current);
                        		finished = true;
                        		break;
                        	}
                            //continue if neighbor is an obstacle or it is contained in the closedList
                            if(neighbor.isObstacle() || dijkstraClosedList.contains(neighbor)) {
                            		continue;
                            }
                            // calculate g(n), the cost to get from start to neighbor node n with the current path
                            float neighborCostFromStart = (current.getCostFromStart() + map.getCostBetween(current, neighbor));
                            //add neighbor to the open list if it is not there or if the new path is shorter 
                            if(!dijkstraOpenList.contains(neighbor) || (neighborCostFromStart < neighbor.getCostFromStart())) {
                        			neighbor.setPreviousNode(current);
                            		neighbor.setCostFromStart(neighborCostFromStart);
                                    neighbor.setFCost(neighborCostFromStart);
                                    if(!dijkstraOpenList.contains(neighbor))
                                    	dijkstraOpenList.add(neighbor, true);
                            } else continue;
                            }

                    }
            		lastOpenList = dijkstraOpenList;
            		lastClosedList = dijkstraClosedList;
            		return dijkstraShortestPath = reconstructPath(current, false, true, false);

        }
        
        /** This is our implementation of the BFS algorithm
         * 	Here we use a QUEUE instead of a SORTED LIST, using the FIFO principle
         * 	We pop nodes from the QUEUE (here: openList) if it is not already visited (if not in closedList) and check if it is the goal
         * 	This algorithm will not necessarily find the shortest path if weights are involved
         * @return
         */
        public Path calcShortestPathUsingBFS() {
        	finished = false;
    		//reset values in nodes that were entered by previous algorithms
    		resetEnteredNodeValues(lastClosedList, lastOpenList);
            map.getStartNode().setCostFromStart(0); //setter avstand fra start til start lik 0
            bfsOpenList.add(map.getStartNode(), false);
            //while we haven't reached the goal yet
            while(!finished) {
            		//get the first Node from non-searched Node list, sorted by lowest distance from our goal as guessed by our heuristic
            		current = bfsOpenList.getLast();
            		//move current Node to the closed (already searched) list
            		bfsOpenList.remove(current);
                	//Initialize current's neighbors ?? or initialize when registering edges?? Or just drop registering edges??
            		map.registerNodeNeighbors(current);
                    bfsClosedList.add(current);
                    //go through all the current Nodes neighbors and calculate if one should be our next step
                    for(Node neighbor : current.getNeighborList()) {
                        	// check if the neighbor is the goal Node. If it is, we are done.
                        	if(neighbor.isGoal()) {
                        		neighbor.setPreviousNode(current);
                        		finished = true;
                        		break;
                        	}
                            //continue if neighbor is an obstacle or it is contained in the closedList
                            if(neighbor.isObstacle() || bfsClosedList.contains(neighbor)) {
                            		continue;
                            }
                            // calculate g(n), the cost to get from start to neighbor node n with the current path
                            float neighborCostFromStart = (current.getCostFromStart() + map.getCostBetween(current, neighbor));
                            //add neighbor to the open list if it is not there or if the new path is shorter 
                            if(!bfsOpenList.contains(neighbor) || (neighborCostFromStart < neighbor.getCostFromStart())) {
                        			neighbor.setPreviousNode(current);
                            		neighbor.setCostFromStart(neighborCostFromStart);
                            		// calculate h(n), the heuristic which is the distance from the neighbor node to the goal
                                    float estimatedDistanceToGoal = heuristic.getEstimatedDistanceToGoal(neighbor.getX(), neighbor.getY(), map.getGoalLocationX(), map.getGoalLocationY());
                                    neighbor.setFCost(neighborCostFromStart + estimatedDistanceToGoal);
                                    if(!bfsOpenList.contains(neighbor))
                                    	bfsOpenList.addAsFirst(neighbor);
                            } else continue;
                            }

                    }
            		lastOpenList = bfsOpenList;
            		lastClosedList = bfsClosedList;
            		return bfsShortestPath = reconstructPath(current, false, false, true);
        }

        //this is our toString() method which returns a String object representing the map and it's shortest path
        public String printPath(boolean aStar, boolean dijkstra, boolean bfs, boolean showEnteredNodes, boolean statistics){
	        	String tempString = "";
	        	String finalString ="";
	        	Node node;
	        	if(aStar){
	        			finalString = finalString + "\n--------------------------------\nA-STAR\n";
			        	for(int x = 0; x < map.getHeightMap(); x++){
			        		for(int y = 0; y < map.getWidthMap(); y++){
			        			node = map.getNode(x, y);
			        			if(aStarShortestPath.contains(node.getX(), node.getY()) && !(node.isGoal() || node.isStart())){
			        				tempString = tempString + "+";
			        			}
			        			//(!"B".equals(Character.toString(node.getCharType())) || "A".equals(Character.toString(node.getCharType()))
			        			else if(showEnteredNodes && aStarOpenList.contains(node) && !(node.isGoal() || node.isStart())){
			        				tempString = tempString + "O";
			        			}
			        			else if(showEnteredNodes && aStarClosedList.contains(node) && !(node.isGoal() || node.isStart())){
			        				tempString = tempString + "X";
			        			}
			        			else{
			        				tempString = tempString + Character.toString(node.getCharType());
			        			}
			        		}
			        		tempString = tempString + "\n";
			        		finalString = finalString + tempString;
			        		tempString = "";
			        	}
	        	}
	        	
	        	
	        	if(dijkstra){
	        		finalString = finalString + "\n--------------------------------\nDIJKSTRA\n";
		        	for(int x = 0; x < map.getHeightMap(); x++){
		        		for(int y = 0; y < map.getWidthMap(); y++){
		        			node = map.getNode(x, y);
		        			if(dijkstraShortestPath.contains(node.getX(), node.getY()) && !(node.isGoal() || node.isStart())){
		        				tempString = tempString + "+";
		        			}
		        			//(!"B".equals(Character.toString(node.getCharType())) || "A".equals(Character.toString(node.getCharType()))
		        			else if(showEnteredNodes && dijkstraOpenList.contains(node) && !(node.isGoal() || node.isStart())){
		        				tempString = tempString + "O";
		        			}
		        			else if(showEnteredNodes && dijkstraClosedList.contains(node) && !(node.isGoal() || node.isStart())){
		        				tempString = tempString + "X";
		        			}
		        			else{
		        				tempString = tempString + Character.toString(node.getCharType());
		        			}
		        			
		        		}
		        		tempString = tempString + "\n";
		        		finalString = finalString + tempString;
		        		tempString = "";
		        	}
	        	}
		       
	        	if(bfs){
	        			finalString = finalString + "\n--------------------------------\nBFS\n";
			        	for(int x = 0; x < map.getHeightMap(); x++){
			        		for(int y = 0; y < map.getWidthMap(); y++){
			        			node = map.getNode(x, y);
			        			if(bfsShortestPath.contains(node.getX(), node.getY()) && !(node.isGoal() || node.isStart())){
			        				tempString = tempString + "+";
			        			}
			        			//(!"B".equals(Character.toString(node.getCharType())) || "A".equals(Character.toString(node.getCharType()))
			        			else if(showEnteredNodes && bfsOpenList.contains(node) && !(node.isGoal() || node.isStart())){
			        				tempString = tempString + "O";
			        			}
			        			else if(showEnteredNodes && bfsClosedList.contains(node) && !(node.isGoal() || node.isStart())){
			        				tempString = tempString + "X";
			        			}
			        			else{
			        				tempString = tempString + Character.toString(node.getCharType());
			        			}
			        			
			        		}
			        		tempString = tempString + "\n";
			        		finalString = finalString + tempString;
			        		tempString = "";
			        	}
		        	}
		        	return finalString + printStatistics(aStar, dijkstra, bfs, statistics);
	    }
        
        //This method backtracks from the goal node to start and generates a path representing the shortest path
        private Path reconstructPath(Node node, boolean aStar, boolean dijkstra, boolean bfs) {
                Path shortestPath = new Path();
                while(!(node.getPreviousNode() == null)) {
                        shortestPath.prependWayPoint(node);
                        if(aStar)
                        	aStarPathCost = aStarPathCost + node.getTerrainCost();
                        if(dijkstra)
                        	dijkstraPathCost = dijkstraPathCost + node.getTerrainCost();
                        if(bfs)
                        	bfsPathCost = bfsPathCost + node.getTerrainCost();
                        node = node.getPreviousNode();
                }
                return shortestPath;
        }
        
        //This method is used to reset changed values in Node objects in case a search algorithm has used the objects
        private void resetEnteredNodeValues(ArrayList<Node> closedList, SortedNodeList openList){
        	for(int i = 0; i < closedList.size(); i++){
        		Node enteredNode = closedList.get(i);
        		enteredNode.setFCost(Integer.MAX_VALUE);
        		enteredNode.getNeighborList().clear();
        		enteredNode.setCostFromStart(Integer.MAX_VALUE);
        		enteredNode.setPreviousNode(null);
        	}
        	
        }

        
        /** This is our own datastructure for keeping a sorted list on the F-Cost of the Node objects
         *  It can also be used as a queue!
         * @author Lars
         *
         */
        private class SortedNodeList {
                private ArrayList<Node> list = new ArrayList<Node>();
                public Node getFirst() {
                		if(list.isEmpty()){
                			System.out.println("List is empty");
                			return null;
                		}
                        return list.get(0);
                }
                
                //can be used if you want the last object from the list
                public Node getLast(){
                	if(list.isEmpty()){
                		System.out.println("List is empty");
                		return null;
                	}
                	return list.get(list.size()-1);
                }
                
                //can be used to add nodes if you want to use it as a queue
                public void addAsFirst(Node node){
                	list.add(0, node);
                }

				public void add(Node node, boolean sort) {
                        list.add(node);
                        if(sort)
                        	Collections.sort(list);
                }

                public void remove(Node n) {
                        list.remove(n);
                }

                public boolean contains(Node n) {
                        return list.contains(n);
                }

				public int size(){
					return list.size();
				}
                
        }
        
        //Prints gathered statistics from the searches
        private String printStatistics(boolean aStar, boolean dijkstra, boolean bfs, boolean statistics){
        	if(!statistics){
        		return "";
        	}
        	String tempString = "\n--------------------------------\nSTATISTICS\n";
        	if(aStar){
        		tempString = tempString + "A-STAR\n";
        		tempString = tempString + "Size of openlist: " + aStarOpenList.size() + "\n";
        		tempString = tempString + "Size of closedlist: " + aStarClosedList.size() + "\n";
        		tempString = tempString + "Total path cost: " + aStarPathCost + "\n";
        		tempString = tempString + "\n";
        	}
        	if(dijkstra){
        		tempString = tempString + "DIJKSTRA\n";
        		tempString = tempString + "Size of openlist: " + dijkstraOpenList.size() + "\n";
        		tempString = tempString + "Size of closedlist: " + dijkstraClosedList.size() + "\n";
        		tempString = tempString + "Total path cost: " + dijkstraPathCost + "\n";
        		tempString = tempString + "\n";
        	}
        	if(bfs){
        		tempString = tempString + "BFS\n";
        		tempString = tempString + "Size of openlist: " + bfsOpenList.size() + "\n";
        		tempString = tempString + "Size of closedlist: " + bfsClosedList.size() + "\n";
        		tempString = tempString + "Total path cost: " + bfsPathCost + "\n";
        		tempString = tempString + "\n";
        	}
        	return tempString;
        }
        
        public Path getAStarShortestPath(){
        	return this.aStarShortestPath;
        } 
        public Path getDijkstraShortestPath(){
        	return this.dijkstraShortestPath;
        }
        public Path getBFSShortestPath(){
        	return this.bfsShortestPath;
        }  
        
}
