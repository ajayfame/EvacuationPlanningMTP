package com.iitb.gise;

import java.util.ArrayList;

public class Edge {

	private Node source;
	private Node target;
	private double travelTime;
	private int maxIntakeCapacity;
	private String edgeName;
	private int edgeID;
	private ArrayList<EdgeCapacityAtThisTime> edgeCapacityAtTimeInstance;	//array of arrays
	
	private ArrayList<Boolean> edgeSourceBlocked;	
	private ArrayList<Boolean> edgeTargetBlocked;
	private EdgeCapacityAtThisTime edgeMaxCapacityTimeInstance;
	private int edgeUsage;
	private ArrayList<Integer> edgeCapacity;
	private ArrayList<Integer> tempEdgeCapacity;	//array of arrays

	
	public ArrayList<Integer> getEdgeCapacity() {
		return edgeCapacity;
	}

	public void setEdgeCapacity(ArrayList<Integer> edgeCapacity) {
		this.edgeCapacity = edgeCapacity;
	}

	public void addEdgeCapacity() {
		this.edgeCapacity.add(maxIntakeCapacity);
	}
	
	public ArrayList<Integer> getTempEdgeCapacity() {
		return tempEdgeCapacity;
	}

	public void setTempEdgeCapacity(ArrayList<Integer> tempEdgeCapacity) {
		this.tempEdgeCapacity = tempEdgeCapacity;
	}

	public void addTempEdgeCapacity() {
		this.tempEdgeCapacity.add(maxIntakeCapacity);
	}
	
	public ArrayList<Boolean> getEdgeSourceBlocked() {
		return edgeSourceBlocked;
	}

	public void setEdgeSourceBlocked(ArrayList<Boolean> edgeSourceBlocked) {
		this.edgeSourceBlocked = edgeSourceBlocked;
	}

	public ArrayList<Boolean> getEdgeTargetBlocked() {
		return edgeTargetBlocked;
	}

	public void setEdgeTargetBlocked(ArrayList<Boolean> edgeTargetBlocked) {
		this.edgeTargetBlocked = edgeTargetBlocked;
	}

	public Node getSource() {
		return source;
	}

	public void setSource(Node source) {
		this.source = source;
	}

	public EdgeCapacityAtThisTime getEdgeMaxCapacityTimeInstance() {
		return edgeMaxCapacityTimeInstance;
	}

	public void setEdgeMaxCapacityTimeInstance(
			EdgeCapacityAtThisTime edgeMaxCapacityTimeInstance) {
		this.edgeMaxCapacityTimeInstance = edgeMaxCapacityTimeInstance;
	}

	public int getEdgeUsage() {
		return edgeUsage;
	}

	public void setEdgeUsage(int edgeUsage) {
		this.edgeUsage = edgeUsage;
	}

	public Node getTarget() {
		return target;
	}

	public void setTarget(Node target) {
		this.target = target;
	}
	
	public double getTravelTime() {
		return travelTime;
	}

	public void setTravelTime(double travelTime) {
		this.travelTime = travelTime;
	}

	public int getMaxIntakeCapacity() {
		return maxIntakeCapacity;
	}

	public void setMaxIntakeCapacity(int maxIntakeCapacity) {
		this.maxIntakeCapacity = maxIntakeCapacity;
	}

	public String getEdgeName() {
		return edgeName;
	}

	public void setEdgeName(String edgeName) {
		this.edgeName = edgeName;
	}

	public int getEdgeID() {
		return edgeID;
	}

	public void setEdgeID(int edgeID) {
		this.edgeID = edgeID;
	}

	public ArrayList<EdgeCapacityAtThisTime> getEdgeCapacityAtTimeInstance() {
		return edgeCapacityAtTimeInstance;
	}

	public void setEdgeCapacityAtTimeInstance(
			ArrayList<EdgeCapacityAtThisTime> edgeCapacityAtTimeInstance) {
		this.edgeCapacityAtTimeInstance = edgeCapacityAtTimeInstance;
	}
	
	public void addEdgeCapacityAtNextTimeInstance(
			EdgeCapacityAtThisTime edgeCapacityAtThisTime) 
	{
		if(this.edgeCapacityAtTimeInstance == null)
			this.edgeCapacityAtTimeInstance = new ArrayList<EdgeCapacityAtThisTime>();
		EdgeCapacityAtThisTime edgeCapacity = new EdgeCapacityAtThisTime(edgeCapacityAtThisTime);
		this.edgeCapacityAtTimeInstance.add(edgeCapacity);
	}
	
	public void printEdgeCapacity()
	{
		for(int i=0;i<this.edgeCapacityAtTimeInstance.size();i++)
		{
			EdgeCapacityAtThisTime e = edgeCapacityAtTimeInstance.get(i);
			for(int j=0;j<e.getCapacityOfSection().size();j++)
			{
				System.out.print(e.getCapacityOfSection().get(j) + " ");
			}
			System.out.println();
		}
	}
}