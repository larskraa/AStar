
package aStar.heuristics;


/* Heuristic - Manhattan distance */
public class ClosestHeuristic implements AStarHeuristic {
        public float getEstimatedDistanceToGoal(int startX, int startY, int goalX, int goalY) {  
        		if((startX == goalX) && (startY == goalY)){
        			return 0;
        		}
                float dx = Math.abs(goalX - startX);
                float dy = Math.abs(goalY - startY);
                
                float result = (float) (dx)+(dy);
                return result;
        }
}