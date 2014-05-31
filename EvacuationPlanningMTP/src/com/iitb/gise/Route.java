package com.iitb.gise;

import java.util.ArrayList;
/*
 * DepartureTime starts from source to destination (-1 for destination)
 * Arrival time starts from  source to destination (0 for source)
 * #edgeEntries - 1 = #departureTimeEntries = #arrivalTimeEntries = #nodeEntries
 */
public class Route implements Comparable<Route>{

	ArrayList<String> nodeList;
	ArrayList<Node> routeNodeList;
	ArrayList<Edge> routeEdgeList;
	ArrayList<String> edgeList;
	ArrayList<Integer> departureTime;
	ArrayList<Integer> arrivalTime;
	ArrayList<Integer> capAtDelay;    ///used only for OneSourceOnePath Algorithm
	int routeId;
	int groupSize;
	double routeTravelTime = 0;
	String routeString = "";
	public static int maxHops = 0;
	public static int totalHops = 0;
	public static double maxLength = 0;
	
	
	public void addCapAtDelay(int cap)
	{
		if(capAtDelay == null)
			capAtDelay = new ArrayList<Integer>();
		capAtDelay.add(cap);
	}
	
	public ArrayList<Integer> getCapAtDelay() {
		return capAtDelay;
	}

	public void setCapAtDelay(ArrayList<Integer> capAtDelay) {
		this.capAtDelay = capAtDelay;
	}

	public static int getTotalHops() {
		return totalHops;
	}

	public static void setTotalHops(int totalHops) {
		Route.totalHops = totalHops;
	}

	public static void addTotalHops(int totalHops) {
		Route.totalHops += totalHops;
	}
	public Route()
	{
		nodeList = new ArrayList<String>();
		edgeList = new ArrayList<String>();
		routeNodeList = new ArrayList<Node>();
		routeEdgeList = new ArrayList<Edge>();
		departureTime = new ArrayList<Integer>();
		arrivalTime = new ArrayList<Integer>();
	}
	
	public ArrayList<Node> getRouteNodeList() {
		return routeNodeList;
	}

	public void setRouteNodeList(ArrayList<Node> routeNodeList) {
		this.routeNodeList = routeNodeList;
	}

	public ArrayList<Edge> getRouteEdgeList() {
		return routeEdgeList;
	}

	public void setRouteEdgeList(ArrayList<Edge> routeEdgeList) {
		this.routeEdgeList = routeEdgeList;
	}

	public ArrayList<String> getNodeList() {
		return nodeList;
	}
	public void setNodeList(ArrayList<String> nodeList) {
		this.nodeList = nodeList;
	}
	public ArrayList<String> getEdgeList() {
		return edgeList;
	}
	public void setEdgeList(ArrayList<String> edgeList) {
		this.edgeList = edgeList;
	}
	public int getRouteId() {
		return routeId;
	}
	public void setRouteId(int routeId) {
		this.routeId = routeId;
	}
	public int getGroupSize() {
		return groupSize;
	}
	public void setGroupSize(int groupSize) {
		this.groupSize = groupSize;
	}
	public ArrayList<Integer> getDepartureTime() {
		return departureTime;
	}
	public void setDepartureTime(ArrayList<Integer> departureTime) {
		this.departureTime = departureTime;
	}
	public ArrayList<Integer> getArrivalTime() {
		return arrivalTime;
	}
	public void setArrivalTime(ArrayList<Integer> arrivalTime) {
		this.arrivalTime = arrivalTime;
	}
	
	public static int getMaxHops() {
		return maxHops;
	}
	public static void setMaxHops(int maxHops) {
		Route.maxHops = maxHops;
	}
	public static double getMaxLength() {
		return maxLength;
	}
	public static void setMaxLength(double maxLength) {
		Route.maxLength = maxLength;
	}
	
	public double getRouteTravelTime() {
		return routeTravelTime;
	}
	public void setRouteTravelTime(double routeTravelTime) {
		this.routeTravelTime = routeTravelTime;
	}
	public String getRouteString() {
		String travelPlan = "";
		int index;
		for(index = 0; index < (nodeList.size() - 2); index++)
		{
			travelPlan += this.getNodeList().get(index) + "---";
		}
		travelPlan += this.getNodeList().get(index);
		return travelPlan;
	}
	public void setRouteString(String routeString) {
		this.routeString = routeString;
	}
	
	public void displayRoute2()
	{
		String plannedPath = this.routeId + "||" + this.groupSize + "||";
		String travelPlan = "";
		int index = 0;
		for(index = 0; index < (nodeList.size() - 1); index++)
		{
			travelPlan += "(" + this.getArrivalTime().get(index) + ")" + this.getNodeList().get(index);
			travelPlan += "(" + this.getDepartureTime().get(index) + ")---" ;//+ this.getEdgeList().get(index) + "]--";
		}
		travelPlan += "[(" + this.getArrivalTime().get(index) + ")" + this.getNodeList().get(index) + "]";
		System.out.println(plannedPath + travelPlan);
	}
	
	public void displayRoute3(int delay, int groupsize)
	{
		String plannedPath = this.routeId + "||" + this.groupSize + "||";
		String travelPlan = "";
		int index = 0;
		for(index = 0; index < (nodeList.size() - 1); index++)
		{
			travelPlan += "(" + (this.getArrivalTime().get(index) + delay) + ")" + this.getNodeList().get(index);
			travelPlan += "(" + (this.getDepartureTime().get(index) + delay) + ")---" ;//+ this.getEdgeList().get(index) + "]--";
		}
		travelPlan += "[(" + (this.getArrivalTime().get(index) + delay) + ")" + this.getNodeList().get(index) + "]";
		System.out.println(plannedPath + travelPlan);
	}
	
	public void displayRoute4(int groupsize)
	{
		String plannedPath = this.routeId + "||" + this.groupSize + "||";
		String travelPlan = "";
		int index = 0;
		travelPlan += groupsize + "---x---" + 
		this.getArrivalTime().get(nodeList.size()-1) + "---";
		for(index = 0; index < (nodeList.size() - 1); index++)
		{
			travelPlan += this.getNodeList().get(index) + "---";
			//travelPlan += "(" + this.getDepartureTime().get(index) + ")" + this.getEdgeList().get(index) + "]--";
		}
		travelPlan += this.getNodeList().get(index);
		System.out.println(travelPlan);
	}
	
	public void displayRoute()
	{
		String plannedPath = this.routeId + "||" + this.groupSize + "||";
		String travelPlan = "";
		int index = 0;
		travelPlan += this.getGroupSize() + "---x---" + 
		this.getArrivalTime().get(nodeList.size()-1) + "---";
		for(index = 0; index < (nodeList.size() - 1); index++)
		{
			travelPlan += this.getNodeList().get(index) + "---";
			//travelPlan += "(" + this.getDepartureTime().get(index) + ")" + this.getEdgeList().get(index) + "]--";
		}
		travelPlan += this.getNodeList().get(index);
		System.out.println(travelPlan);
	}
	@Override
	public int compareTo(Route otherRoute) {
		if(this.getRouteTravelTime() < otherRoute.getRouteTravelTime())
		{
			return -1;
		}
		else if(this.getRouteTravelTime() > otherRoute.getRouteTravelTime())
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
}