package aStar;


import aStar.heuristics.AStarHeuristic;
import aStar.heuristics.ClosestHeuristic;
import aStar.utils.Logger;
import aStar.utils.StopWatch;

public class TestAStar {
       
		// Full local path to boards is contained in the string "path"
       	private static String path = "/Users/Lars/Google Drive/NTNU/FJERDE KLASSE/AI/Øvinger/Assignment3/boards/board-2-4.txt";
   
        public static void main(String[] args) {
                Logger log = new Logger();
                StopWatch s = new StopWatch();
                s.start();
                AreaMap map = new AreaMap(path);                
                AStarHeuristic heuristic = new ClosestHeuristic();
                ShortestPathAlgorithm pathFinder = new ShortestPathAlgorithm(map, heuristic);
                pathFinder.calcShortestPathUsingAStar();
                pathFinder.calcShortestPathUsingDijkstra();
                pathFinder.calcShortestPathUsingBFS();
                s.stop();
                log.addToLog("Time to calculate path in milliseconds: " + s.getElapsedTime());
                
                log.addToLog("Printing map of shortest path...");
                /* The printPath method i used to print the different boards. You can also activate printing of visited nodes and statistics from the searches.
                 * The boolean attributes are explained below.
                 * printPath(print result from A-Star, print result from Dijkstra, print result from BFS, print elements in open/closed list, print statistics) */
                System.out.println(pathFinder.printPath(true, true, true, true, true));
                
        
        }

        
        
        

}
