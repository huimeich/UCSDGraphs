/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Set;
import java.util.Map;
import java.util.Queue;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraphNode {
	//TODO: Add your member variables here in WEEK 2
	private GeographicPoint location;
	private List<MapGraphNode> neighbors;
	private List<String> roadName;
	private List<String> roadType;
	private List<Double> length;	
	private double distance;

	public MapGraphNode(){
		location = null;
		neighbors = new ArrayList<MapGraphNode>();
		roadName = new ArrayList<String>();
		roadType = new ArrayList<String>();
		length = new ArrayList<Double>();
		distance = Double.MAX_VALUE;
	}
	
	public MapGraphNode(GeographicPoint location){
		this.location = location;
		this.neighbors = new ArrayList<MapGraphNode>();
		this.roadName = new ArrayList<String>();
		this.roadType = new ArrayList<String>();
		this.length = new ArrayList<Double>();
		this.distance = Double.MAX_VALUE;
	}
	
	//@Override
	public boolean equals(Object other){
		boolean isSame = false;
		if (other != null && other instanceof MapGraphNode) {
			MapGraphNode node = (MapGraphNode) other;
			isSame = this.location.equals( ((MapGraphNode) node).getLocation() );
		}
		return isSame;
	}

	public void addNeighbor(MapGraphNode neighbor, String roadName, String roadType, Double length) {
		(this.neighbors).add(neighbor);
		(this.roadName).add(roadName);
		(this.roadType).add(roadType);
		(this.length).add(length);
	}
	
	/**
	 * @return the neighbors
	 */
	public List<MapGraphNode> getNeighbors() {
		return this.neighbors;
	}

	public GeographicPoint getLocation() {
		return location;
	}
	
	public double getLength(MapGraphNode to){
		if (this.equals(to)) {
			return 0.0;
		}
		if (neighbors.contains(to)) {
			return length.get(neighbors.indexOf(to));
		}
//		for (MapGraphNode v: neighbors) {
//				System.out.println(v.getLocation().toString());
//		}
		System.out.println("Wrong! No " + to.getLocation().toString() + " in neighbors" + this.getLocation().toString());
		return -1;
	}
	
	public double getDistance(){
		return distance;
	}
	public void changeDistance(double newDistance){
		this.distance = newDistance;
	}
	
}
