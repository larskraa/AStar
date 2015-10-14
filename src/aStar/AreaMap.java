package aStar;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

/** This class holds the map representing the terrain.
 * 	It contains methods for reading the map from a txt file, analyzing the map and generating nodes and their attributes
 * 	It also contains methods for managing the map and entering the nodes in the map for further analysis like finding the shortest path
 *  Also contains important map properties like goal location, start location, height and width of the map, etc.
 * 
 * @author Lars
 *
 */


public class AreaMap {

        private int heightMap;
        private int widthMap;
        private ArrayList<ArrayList<Node>> map;
        private int startLocationX;
        private int startLocationY;
        private int goalLocationX;
        private int goalLocationY;
        private Character[][] obstacleMap;
        private BufferedReader mapReader;
        
        AreaMap(String path) {
        		// Reads and initializes the map
                try {
					calcMapProperties(path, mapReader = new BufferedReader(new FileReader(path)));
				} catch (IOException e) {
					System.out.println("Could read map from path or calculate map properties.");
					e.printStackTrace();
				} finally{
					try {
						mapReader.close();
					} catch (IOException e) {
						System.out.println("Could not close BufferedReader after calcMapProperties.");
						e.printStackTrace();
					}
				}
                
                try {
					obstacleMap = readMap(path, mapReader = new BufferedReader(new FileReader(path)), heightMap, widthMap);
				} catch (FileNotFoundException e) {
					System.out.println("Could not find file in path.");
					e.printStackTrace();
				} catch (IOException e) {
					System.out.println("Could not read from file.");
					e.printStackTrace();
				} finally{
					try {
						mapReader.close();
					} catch (IOException e) {
						System.out.println("Could not close BufferedReader after generating the obstacleMap");
						e.printStackTrace();
					}
				}
                createMap();
        }
        
        //generates a 2-dimensional grid of nodes representing the board/map
        private void createMap() {
                Node node;
                map = new ArrayList<ArrayList<Node>>();
                for (int x = 0; x < heightMap; x++) {
                        map.add(new ArrayList<Node>());
                        for (int y = 0; y < widthMap; y++){
                        		node = new Node(x,y);
                                if ("#".equals(Character.toString(obstacleMap[x][y]))){
                                    node.setObstical(true);
                                	node.setCharType('#');
                        		}
                                else if("w".equals(Character.toString(obstacleMap[x][y]))){
                                	node.setTerrainCost(Costs.w.getCost());
                                	node.setCharType('~');
                                }
                                else if("m".equals(Character.toString(obstacleMap[x][y]))){
                                	node.setTerrainCost(Costs.m.getCost());
                                	node.setCharType('/');
                                }
                                else if("f".equals(Character.toString(obstacleMap[x][y]))){
                                	node.setTerrainCost(Costs.f.getCost());
                                	node.setCharType('#');
                                }
                                else if("g".equals(Character.toString(obstacleMap[x][y]))){
                                	node.setTerrainCost(Costs.g.getCost());
                                	node.setCharType('_');
                                }
                                else if("r".equals(Character.toString(obstacleMap[x][y]))){
                                	node.setTerrainCost(Costs.r.getCost());
                                	node.setCharType('=');
                                }
                                else if(".".equals(Character.toString(obstacleMap[x][y]))){
                                	node.setTerrainCost(Costs.r.getCost());
                                	node.setCharType('.');
                                }
                                else if("A".equals(Character.toString(obstacleMap[x][y]))){
                                	node.setStart(true);
                                	node.setCharType('A');
                                	startLocationX = x;
                                	startLocationY = y;
                                }
                                else if("B".equals(Character.toString(obstacleMap[x][y]))){
                                	node.setGoal(true);
                                	node.setCharType('B');
                                	goalLocationX = x;
                                	goalLocationY = y;
                                }
                                else{
                                	System.out.println("Could not evaluate obstacle in the map at coordinate (" + x + "," + y + ")");
                                }
                                map.get(x).add(node);
                        }
                }
        }
        
        //Initialize map properties for later use, like height and width of the map
        private void calcMapProperties(String path, BufferedReader mapReader) throws IOException{
    		heightMap = 0;
    		String row = mapReader.readLine();
    		widthMap = row.length();
    		while(row != null){
    			heightMap++;
    			row = mapReader.readLine();
    		}
    	}
        
        //reads the map and creates a 2-dimensional character array of the map
        public Character[][] readMap(String path, BufferedReader mapReader, int heightMap, int widthMap) throws IOException{
        	Character[][] map = new Character[heightMap][widthMap];
        	String row = mapReader.readLine();
        	int height = 0;
        	while(row != null && height < heightMap){
        		for(int y=0; y < widthMap; y++){
        			map[height][y] = row.charAt(y);
        		}
        		height++;
        		row = mapReader.readLine();
        	}
			return map;
        }
     
        //Registers neighbor nodes to the parameter node
       public void registerNodeNeighbors(Node node){
        	if(!(node.getX()==0) && !node.getNeighborList().contains(map.get(node.getX()-1).get(node.getY())))
            	node.getNeighborList().add((map.get(node.getX()-1).get(node.getY())));
            if(!(node.getY()>=widthMap-1) && !node.getNeighborList().contains(map.get(node.getX()).get(node.getY()+1)))
            	node.getNeighborList().add(map.get(node.getX()).get(node.getY()+1));
            if(!(node.getX()>=heightMap-1) && !node.getNeighborList().contains(map.get(node.getX()+1).get(node.getY())))
            	node.getNeighborList().add(map.get(node.getX()+1).get(node.getY()));
            if(!(node.getY()==0) && !node.getNeighborList().contains(map.get(node.getX()).get(node.getY()-1)))
            	node.getNeighborList().add(map.get(node.getX()).get(node.getY()-1));
        }

        public ArrayList<ArrayList<Node>> getMap() {
                return map;
        }
        public void setObstical(int x, int y, boolean isObstical) {
                map.get(x).get(y).setObstical(isObstical);
        }

        public Node getNode(int x, int y) {
                return map.get(x).get(y);
        }

        public int getStartLocationX() {
                return startLocationX;
        }

        public int getStartLocationY() {
                return startLocationY;
        }
        
        public Node getStartNode() {
                return map.get(startLocationX).get(startLocationY);
        }

        public int getGoalLocationX() {
                return goalLocationX;
        }

        public int getGoalLocationY() {
                return goalLocationY;
        }
        
        
        public float getCostBetween(Node node1, Node node2) {
        		if(node1.isNeighborTo(node2)){
        			return node2.getTerrainCost();
        		}
        		else return -Integer.MAX_VALUE;
               	
        }
        
        public int getHeightMap() {
                return heightMap;
        }
        public int getWidthMap() {
                return widthMap;
        }
        
   
        
}
