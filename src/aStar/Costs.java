package aStar;

/**
 * Here we created an enum for easier management of constant variables like terrain costs and their corresponding symbols
 * @author Lars
 *
 */

public enum Costs {
	
	w(100), m(50), f(10), g(5), r(1);
	private int cost;
	
	private Costs(int cost){
		this.cost = cost;
	}
	
	public int getCost(){
		return cost;
	}
	
	
}
