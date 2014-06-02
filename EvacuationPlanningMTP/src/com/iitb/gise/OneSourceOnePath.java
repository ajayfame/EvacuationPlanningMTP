package com.iitb.gise;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

public class OneSourceOnePath {

	private Graph graph = new Graph();
	private Set<Node> PQAdditionList = new HashSet<Node>();
	private static double humanWalkingSpeed = 1.5;			//in m/s
	private int pathId = 1;
	private ArrayList<Route> pathList = new ArrayList<Route>();
	private Set<String> distinctRoutes = new LinkedHashSet<String>();
	List<Node> srcList = new ArrayList<Node>();
	int evacuationTime = 0;
	
	public Set<String> getDistinctRoutes() {
		return distinctRoutes;
	}

	public void setDistinctRoutes(Set<String> distinctRoutes) {
		this.distinctRoutes = distinctRoutes;
	}

	public int getPathId() {
		return pathId;
	}

	public void setPathId(int pathId) {
		this.pathId = pathId;
	}

	public ArrayList<Route> getPathList() {
		return pathList;
	}

	public void setPathList(ArrayList<Route> pathList) {
		this.pathList = pathList;
	}

	public Set<Node> getPQAdditionList() {
		return PQAdditionList;
	}

	public void setPQAdditionList(Set<Node> pQAdditionList) {
		PQAdditionList = pQAdditionList;
	}

	public static double getHumanWalkingSpeed() {
		return humanWalkingSpeed;
	}

	public static void setHumanWalkingSpeed(double humanWalkingSpeed) {
		OneSourceOnePath.humanWalkingSpeed = humanWalkingSpeed;
	}

	public Graph getGraph() {
		return graph;
	}

	public void setGraph(Graph graph) {
		this.graph = graph;
	}

	//finding the vertex by name
	public Node getNode(String nodeName) {
		Iterator<Node> it = this.getGraph().getNodeList().iterator();
		while (it.hasNext()) 
		{
			// System.out.println("getNode : 1");
			Node temp = it.next();
			if (temp.getNodeName().equals(nodeName))
				return temp;
		}
		return null;
	}
	
	public void addNodeToGraph(String nodeId, String nodeName, double x, double y, 
			int maxCapacity, int initialOccupancy, int nodeType)
	{
		Node node = new Node();
		node.setNodeId(nodeId);
		node.setNodeName(nodeName);
		node.setX(x);
		node.setY(y);
		node.setMaxCapacity(maxCapacity);
		node.setInitialOccupancy(initialOccupancy);
		node.setCurrentOccupancy(initialOccupancy);
		node.setNodeType(nodeType);
		node.setAdjacencies(null);
		node.setAdjacentScannedList(null);
		node.setParent(null);
		node.setPathUptoPreviousNode(null);
		node.setScanned(false);
		node.setChildList(null);
		if(nodeType == Node.DESTINATION)
			node.setMaxCapacity(Integer.MAX_VALUE);
		if(nodeType == Node.SOURCE)
		{
			node.setTravelTime(0);
			srcList.add(node);
		}
		else
			node.setTravelTime(Double.MAX_VALUE);
		ArrayList<Integer> nodeCapacityAtTime = new ArrayList<Integer>();
		nodeCapacityAtTime.add(node.getMaxCapacity());
		node.setNodeCapacityAtTime(nodeCapacityAtTime);
		this.getGraph().addNode(node);
	}
	
	public void addNodeToGraph(String nodeId, String nodeName, 
			int maxCapacity, int initialOccupancy, int nodeType)
	{
		Node node = new Node();
		node.setNodeId(nodeId);
		node.setNodeName(nodeName);
		node.setMaxCapacity(maxCapacity);
		node.setInitialOccupancy(initialOccupancy);
		node.setCurrentOccupancy(initialOccupancy);
		node.setNodeType(nodeType);
		node.setAdjacencies(null);
		node.setAdjacentScannedList(null);
		node.setParent(null);
		node.setPathUptoPreviousNode(null);
		node.setScanned(false);
		node.setChildList(null);
		if(nodeType == Node.DESTINATION)
			node.setMaxCapacity(Integer.MAX_VALUE);
		if(nodeType == Node.SOURCE)
		{
			node.setTravelTime(0);
			srcList.add(node);
		}
		else
			node.setTravelTime(Double.MAX_VALUE);
		ArrayList<Integer> nodeCapacityAtTime = new ArrayList<Integer>();
		nodeCapacityAtTime.add(node.getMaxCapacity());
		node.setNodeCapacityAtTime(nodeCapacityAtTime);
		this.getGraph().addNode(node);
	}
	
	public void addEdgeToGraph(int edgeID, String edgeName, String sourceName, 
			String targetName, int maxIntakeCapacity)
	{
		Edge edge = new Edge();
		edge.setEdgeID(edgeID);
		edge.setEdgeName(edgeName);
		edge.setMaxIntakeCapacity(maxIntakeCapacity);
		Node src = this.getNode(sourceName);
		Node target = this.getNode(targetName);
		int travelTime = (int) Math.ceil(Node.calculateDistance(src, target)/humanWalkingSpeed);
		edge.setTravelTime(travelTime);
		edge.setSource(src);
		edge.setTarget(target);
		
		EdgeCapacityAtThisTime edgeCapacityAtThisTime = new EdgeCapacityAtThisTime();
		edge.setEdgeCapacity(new ArrayList<Integer>());
		
		//For each section of edge, initially(t=0) capacity is its maximum
		for (int i = 0; i < travelTime; i++)
		{
			edgeCapacityAtThisTime.getCapacityOfSection().add(maxIntakeCapacity);
			edge.getEdgeCapacity().add(maxIntakeCapacity);
		}
		
		edge.addEdgeCapacityAtNextTimeInstance(edgeCapacityAtThisTime);
		EdgeCapacityAtThisTime edgeMaxCapacityTime = new EdgeCapacityAtThisTime(edgeCapacityAtThisTime);
		edge.setEdgeMaxCapacityTimeInstance(edgeMaxCapacityTime);
		graph.addEdge(edge);
		
		src.addAdjacentEdge(edge);
		target.addAdjacentEdge(edge);
	}
	
	public void addEdgeToGraph(int edgeID, String edgeName, String sourceName, 
			String targetName, int maxIntakeCapacity, int travelTime)
	{
		Edge edge = new Edge();
		edge.setEdgeID(edgeID);
		edge.setEdgeName(edgeName);
		edge.setMaxIntakeCapacity(maxIntakeCapacity);
		Node src = this.getNode(sourceName);
		Node target = this.getNode(targetName);
		edge.setTravelTime(travelTime);
		edge.setSource(src);
		edge.setTarget(target);
		
		EdgeCapacityAtThisTime edgeCapacityAtThisTime = new EdgeCapacityAtThisTime();
		edge.setEdgeCapacity(new ArrayList<Integer>());
		
		//For each section of edge, initially(t=0) capacity is its maximum
		for (int i = 0; i < travelTime; i++)
		{
			edgeCapacityAtThisTime.getCapacityOfSection().add(maxIntakeCapacity);
			edge.getEdgeCapacity().add(maxIntakeCapacity);
		}
		
		edge.addEdgeCapacityAtNextTimeInstance(edgeCapacityAtThisTime);
		EdgeCapacityAtThisTime edgeMaxCapacityTime = new EdgeCapacityAtThisTime(edgeCapacityAtThisTime);
		edge.setEdgeMaxCapacityTimeInstance(edgeMaxCapacityTime);
		graph.addEdge(edge);
		//System.out.println(src.getNodeName());
		src.addAdjacentEdge(edge);
		target.addAdjacentEdge(edge);
	}
	
	public void addEdgeToGraph(int edgeID, String edgeName, String sourceName, 
			String targetName, int maxIntakeCapacity, int travelTime, int x)
	{
		Edge edge = new Edge();
		edge.setEdgeID(edgeID);
		edge.setEdgeName(edgeName);
		edge.setMaxIntakeCapacity(maxIntakeCapacity);
		Node src = this.getNode(sourceName);
		Node target = this.getNode(targetName);
		edge.setTravelTime(travelTime);
		edge.setSource(src);
		edge.setTarget(target);
		
		EdgeCapacityAtThisTime edgeCapacityAtThisTime = new EdgeCapacityAtThisTime();
		edge.setEdgeCapacity(new ArrayList<Integer>());
		
		//For each section of edge, initially(t=0) capacity is its maximum
		for (int i = 0; i < travelTime; i++)
		{
			edgeCapacityAtThisTime.getCapacityOfSection().add(maxIntakeCapacity);
			edge.getEdgeCapacity().add(maxIntakeCapacity);
		}
		
		edge.addEdgeCapacityAtNextTimeInstance(edgeCapacityAtThisTime);
		EdgeCapacityAtThisTime edgeMaxCapacityTime = new EdgeCapacityAtThisTime(edgeCapacityAtThisTime);
		edge.setEdgeMaxCapacityTimeInstance(edgeMaxCapacityTime);
		graph.addEdge(edge);
		
		src.addAdjacentEdge(edge);
		//target.addAdjacentEdge(edge);
	}
	
	
	public void oneSourceOnePathPlanner()
	{
		Comparator<Route> comp = new Comparator<Route>() {

			@Override
			public int compare(Route o1, Route o2) {
				// TODO Auto-generated method stub
				if(o1.getRouteTravelTime() > o2.getRouteTravelTime())
				{
					return -1;
				}
				return 1;
			}
		};
		Route.setTotalHops(0);
		//For each source find the shortest available path to a destination.

		PriorityQueue<Route> preRQ = new PriorityQueue<Route>(srcList.size(), comp);

		int totalEvacuees = 0;
		System.out.println("A1");
		for(int i=0;i<srcList.size();i++)
		{
			Node src = srcList.get(i);
			totalEvacuees += src.getInitialOccupancy();
			clearNodeData(src);
			Route r = shortestPath(src);
			preRQ.add(r);
		}
		System.out.println("A2");

		double totalEgressTime = 0;
		while(totalEvacuees > 0)
		{
			Route r = preRQ.poll();
			totalEgressTime = reserveAllPaths(r);
			Node src = r.getRouteNodeList().get(0);
			src.setNodeType(Node.NORMAL);
			src.setCurrentOccupancy(0);
			totalEvacuees -= src.getInitialOccupancy();
			if(totalEvacuees == 0)
				break;
			srcList.remove(src);
			preRQ.clear();
			for(int i=0;i<srcList.size();i++)
			{
				src = srcList.get(i);
				clearNodeData(src);
				Route rr = shortestPath(src);
				preRQ.add(rr);
			}
		}
		
		System.out.println("Egress Time : " + totalEgressTime);
		System.out.println("RouteList Size : " + pathList.size());
		//System.out.println("Average Evacuation Time : " + 1.0*evacuationTime/count);
		System.out.println("Avg Hops : " + 1.0*Route.getTotalHops()/pathList.size());
		System.out.println("Max Hops : " + Route.getMaxHops());
		System.out.println("No of Distinct Routes : " + this.getDistinctRoutes().size());
		int maxWaitingTimeAtANode = 0;
		String nodeName;
		double totalWaitingTime = 0;
		for(int i=0;i<graph.getNodeList().size();i++)
		{
			totalWaitingTime += graph.getNodeList().get(i).getWaitingTimeAtThisNode();
			if(graph.getNodeList().get(i).getWaitingTimeAtThisNode() > maxWaitingTimeAtANode)
			{
				maxWaitingTimeAtANode = graph.getNodeList().get(i).getWaitingTimeAtThisNode();
				nodeName = graph.getNodeList().get(i).getNodeName();
			}
		}
		System.out.println("Max. Waiting Time at a node : " + maxWaitingTimeAtANode);
		System.out.println("Average. Waiting Time at a node : " + totalWaitingTime/graph.getNodeList().size());
		//System.out.println("Count:" + count);
	}
	
	
	public double reserveAllPaths(Route route)
	{
		Node src = route.getRouteNodeList().get(0);
		System.out.println(src.getNodeName() + "  ++++");
		int evacuees = src.getCurrentOccupancy();
		int delay = 0;
		while(evacuees > 0)
		{
			int groupsize = route.getCapAtDelay().get(delay);
			if(groupsize == 0)
			{
				delay++;
				continue;
			}
			if(evacuees < groupsize)
				groupsize = evacuees;
			reservePath(route, groupsize, delay);
			route.displayRoute3(delay, groupsize);
			delay++;
			
			evacuees -= groupsize;
		}
		return route.getRouteTravelTime();
	}
	public void reservePath(Route route, int groupSize, int delay)
	{
		//PathUptoNode pathUptoNode = destination.getPathUptoPreviousNode();
		System.out.println("Delay:" + delay);
		int noOfNodes = route.getRouteNodeList().size()-1;
		//System.out.println(noOfNodes + "mmmm");
		for(int n=0; n<noOfNodes; n++)
		{
			Node tempNode = route.getRouteNodeList().get(n);
			Edge tempEdge = route.getRouteEdgeList().get(n);
			
			int depart = route.getDepartureTime().get(n);
			int edgeTravelTime = (int)Math.ceil(tempEdge.getTravelTime());
			boolean sameFlowDirection = false;
			
			if(tempNode == tempEdge.getSource())
				sameFlowDirection = true;
			//Edge reservation
			int newCapacity = 0;
			
			if(sameFlowDirection)
			{
				newCapacity = tempEdge.getEdgeCapacity().get(depart + delay) - groupSize;
				tempEdge.getEdgeCapacity().set(depart + delay, newCapacity);
			}
			else
			{
				newCapacity = tempEdge.getEdgeCapacity().get(depart + delay + edgeTravelTime) - groupSize;
				tempEdge.getEdgeCapacity().set(depart + delay + edgeTravelTime, newCapacity);
			}
			//For eg : Departure time at 15, travel time = 5, Hence slot 'time' should be 
			//booked at time 15 + time(0,1,2,3,4) = (15,16,17,18,19)
			
			Node currNode = route.getRouteNodeList().get(n);
			currNode.setNoOfPathsThroughThisNode(currNode.getNoOfPathsThroughThisNode() + 1);
			currNode.setNoOfPeopleThroughThisNode(currNode.getNoOfPeopleThroughThisNode() + groupSize);
			
			int currNodeArrival = 0;
			if(n!=0)
				currNodeArrival = route.getArrivalTime().get(n);
			
			//System.out.print(currNode.getNodeName() + "**" + currNode.getTravelTime());
			int departure = route.getDepartureTime().get(n);
			currNode.setWaitingTimeAtThisNode(departure-currNodeArrival);
			
			for(int i=(currNodeArrival + delay);i<(departure + delay);i++)
			{
				System.out.println(currNodeArrival + " " + delay);
				int newNodeCapacity = tempNode.getNodeCapacityAtTime().get(i) - groupSize;
				tempNode.getNodeCapacityAtTime().set(i, newNodeCapacity);
			}
		}
		
		Node src = route.getRouteNodeList().get(0);
		src.setCurrentOccupancy(src.getCurrentOccupancy() - groupSize);
		
		//Add path to pathList
		this.pathList.add(route);
		
		Route.addTotalHops(route.getNodeList().size());
		if(Route.getMaxHops() < route.getNodeList().size())
			Route.setMaxHops(route.getNodeList().size());
		
		distinctRoutes.add(route.getRouteString());
		this.pathId++;
	}
	
	public int findMinimumCapacity(Route route)
	{
		int noOfNodes = route.getRouteNodeList().size()-1;
		int minCapacity = Integer.MAX_VALUE;
		//without destination
		for(int n=0; n<noOfNodes; n++)
		{
			Node tempNode = route.getRouteNodeList().get(n);
			Edge tempEdge = route.getRouteEdgeList().get(n);
			int depart = route.getDepartureTime().get(n);
			int edgeCapacity = Integer.MAX_VALUE;
			int edgeTravelTime = (int)Math.ceil(tempEdge.getTravelTime());
			int nodeCapacity = Integer.MAX_VALUE;
			boolean sameFlowDirection = false;
			if(tempNode==tempEdge.getSource())
				sameFlowDirection = true;
			
			//For eg : Departure time at 15, travel time = 5, Hence check capacity booked at  
			//slot 'time' for time 15 + time(0,1,2,3,4) = (15,16,17,18,19)
			int newCapacity = 0;
			if(sameFlowDirection)
			{
				newCapacity = tempEdge.getEdgeCapacity().get(depart);
			}
			else
			{
				newCapacity = tempEdge.getEdgeCapacity().get(depart + edgeTravelTime);
			}
			
			if(newCapacity < edgeCapacity)
			{
				edgeCapacity = newCapacity;
			}
			if(n != (noOfNodes - 1))
			{
				tempNode = route.getRouteNodeList().get(n+1);
				int arrival = (int)Math.ceil(tempNode.getTravelTime());
				nodeCapacity = tempNode.getNodeCapacityAtTime().get(arrival);
			}
			if(edgeCapacity < minCapacity && edgeCapacity <= nodeCapacity)
			{
				minCapacity = edgeCapacity;
			}
			else if(nodeCapacity < minCapacity && nodeCapacity < edgeCapacity)
			{
				minCapacity = nodeCapacity;
			}
		}
		if(route.getRouteNodeList().get(0).getCurrentOccupancy() < minCapacity)
			minCapacity = route.getRouteNodeList().get(0).getCurrentOccupancy();
		route.setGroupSize(minCapacity);
		return minCapacity;
	}
	
	
	public boolean check(Route p, Route q)
	{
		if(p.getRouteTravelTime() != q.getRouteTravelTime())
			return false;
		
		if(p.getRouteNodeList().size() != q.getRouteNodeList().size())
			return false;
		
		for(int i=0;i<p.getRouteNodeList().size();i++)
		{
			if(p.getRouteNodeList().get(i) != q.getRouteNodeList().get(i))
				return false;
		}
		return true;
	}
	public void clearNodeData(Node src)
	{
		for(int i=0;i<graph.getNodeList().size();i++)
		{
			
			Node n = graph.getNodeList().get(i);
			n.setChildList(null);
			n.setParent(null);
			n.setPathUptoPreviousNode(null);
			n.setScanned(false);
			n.setAdjacentScannedList(null);
			if(n == src)
				n.setTravelTime(0);
			else
				n.setTravelTime(Double.MAX_VALUE);
		}
	}
	
	public int findMinimumCapacityAlongRoute(Route route, int delay)
	{
		int noOfNodes = route.getRouteNodeList().size()-1;
		int minCapacity = Integer.MAX_VALUE;
		//without destination
		for(int n=0; n<noOfNodes; n++)
		{
			Node tempNode = route.getRouteNodeList().get(n);
			Edge tempEdge = route.getRouteEdgeList().get(n);
			int depart = route.getDepartureTime().get(n);
			int edgeCapacity = Integer.MAX_VALUE;
			int edgeTravelTime = (int)Math.ceil(tempEdge.getTravelTime());
			int nodeCapacity = Integer.MAX_VALUE;
			boolean sameFlowDirection = false;
			if(tempNode==tempEdge.getSource())
				sameFlowDirection = true;
			
			//For eg : Departure time at 15, travel time = 5, Hence check capacity booked at  
			//slot 'time' for time 15 + time(0,1,2,3,4) = (15,16,17,18,19)
			int newCapacity = 0;
			if(sameFlowDirection)
			{
				double timeInstancesToAdd = depart + delay - tempEdge.getEdgeCapacity().size() + 2;

				for(int i = 0; i < timeInstancesToAdd; i++)
					tempEdge.addEdgeCapacity();
				
				newCapacity = tempEdge.getEdgeCapacity().get(depart + delay);
			}
			else
			{
				double timeInstancesToAdd = depart + delay + edgeTravelTime 
						- tempEdge.getEdgeCapacity().size() + 2;

				for(int i = 0; i < timeInstancesToAdd; i++)
					tempEdge.addEdgeCapacity();
				
				newCapacity = tempEdge.getEdgeCapacity().get(depart + 
						delay + edgeTravelTime);
			}
			
			if(newCapacity < edgeCapacity)
			{
				edgeCapacity = newCapacity;
			}
			if(n != (noOfNodes - 1))
			{
				tempNode = route.getRouteNodeList().get(n+1);
				int arrival = (int)Math.ceil(tempNode.getTravelTime());
				double timeInstancesToAdd = arrival + delay - tempNode.getNodeCapacityAtTime().size() + 2;

				for(int i = 0; i < timeInstancesToAdd; i++)
					tempNode.getNodeCapacityAtTime().add(tempNode.getMaxCapacity());
				
				nodeCapacity = tempNode.getNodeCapacityAtTime().get(arrival + delay);
			}
			if(edgeCapacity < minCapacity && edgeCapacity <= nodeCapacity)
			{
				minCapacity = edgeCapacity;
			}
			else if(nodeCapacity < minCapacity && nodeCapacity < edgeCapacity)
			{
				minCapacity = nodeCapacity;
			}
		}
		if(route.getRouteNodeList().get(0).getCurrentOccupancy() < minCapacity)
			minCapacity = route.getRouteNodeList().get(0).getCurrentOccupancy();
		route.setGroupSize(minCapacity);
		return minCapacity;
	}
	
	public void copyActualCapToTemp(Route r)
	{
		for(int i=0;i<r.getRouteNodeList().size();i++)
		{
			Node n = r.getRouteNodeList().get(i);
			n.setTempNodeCapacityAtTime(null);
			n.setTempNodeCapacityAtTime(new ArrayList<Integer>());
			for(int j=0;j<n.getNodeCapacityAtTime().size();j++)
			{
				n.getTempNodeCapacityAtTime().add(n.getNodeCapacityAtTime().get(j));
			}
		}
		for(int i=0;i<r.getRouteEdgeList().size();i++)
		{
			Edge e = r.getRouteEdgeList().get(i);
			e.setTempEdgeCapacity(null);
			e.setTempEdgeCapacity(new ArrayList<Integer>());
			for(int j=0;j<e.getEdgeCapacity().size();j++)
			{
				e.getTempEdgeCapacity().add(e.getEdgeCapacity().get(j));
			}
		}
	}
	
	public void reserveTempPath(Route route, int groupSize, int delay)
	{
		//PathUptoNode pathUptoNode = destination.getPathUptoPreviousNode();
		System.out.println("Delay:" + delay);
		int noOfNodes = route.getRouteNodeList().size()-1;
		//System.out.println(noOfNodes + "mmmm");
		for(int n=0; n<noOfNodes; n++)
		{
			Node tempNode = route.getRouteNodeList().get(n);
			Edge tempEdge = route.getRouteEdgeList().get(n);
			
			int depart = route.getDepartureTime().get(n);
			int edgeTravelTime = (int)Math.ceil(tempEdge.getTravelTime());
			boolean sameFlowDirection = false;
			
			if(tempNode == tempEdge.getSource())
				sameFlowDirection = true;
			//Edge reservation
			int newCapacity = 0;
			
			if(sameFlowDirection)
			{
				newCapacity = tempEdge.getEdgeCapacity().get(depart + delay) - groupSize;
				tempEdge.getEdgeCapacity().set(depart + delay, newCapacity);
			}
			else
			{
				newCapacity = tempEdge.getEdgeCapacity().get(depart + delay + edgeTravelTime) - groupSize;
				tempEdge.getEdgeCapacity().set(depart + delay + edgeTravelTime, newCapacity);
			}
			//For eg : Departure time at 15, travel time = 5, Hence slot 'time' should be 
			//booked at time 15 + time(0,1,2,3,4) = (15,16,17,18,19)
			
			Node currNode = route.getRouteNodeList().get(n);
			currNode.setNoOfPathsThroughThisNode(currNode.getNoOfPathsThroughThisNode() + 1);
			currNode.setNoOfPeopleThroughThisNode(currNode.getNoOfPeopleThroughThisNode() + groupSize);
			
			int currNodeArrival = 0;
			if(n!=0)
				currNodeArrival = route.getArrivalTime().get(n);
			
			//System.out.print(currNode.getNodeName() + "**" + currNode.getTravelTime());
			int departure = route.getDepartureTime().get(n);
			currNode.setWaitingTimeAtThisNode(departure-currNodeArrival);
			
			for(int i=(currNodeArrival + delay);i<(departure + delay);i++)
			{
				System.out.println(currNodeArrival + " " + delay);
				int newNodeCapacity = tempNode.getNodeCapacityAtTime().get(i) - groupSize;
				tempNode.getNodeCapacityAtTime().set(i, newNodeCapacity);
			}
		}
		
		Node src = route.getRouteNodeList().get(0);
		src.setCurrentOccupancy(src.getCurrentOccupancy() - groupSize);
		
		//Add path to pathList
		this.pathList.add(route);
		
		Route.addTotalHops(route.getNodeList().size());
		if(Route.getMaxHops() < route.getNodeList().size())
			Route.setMaxHops(route.getNodeList().size());
		
		distinctRoutes.add(route.getRouteString());
		this.pathId++;
	}
	
	public int findMinimumTempCapacity(Route route)
	{
		int noOfNodes = route.getRouteNodeList().size()-1;
		int minCapacity = Integer.MAX_VALUE;
		//without destination
		for(int n=0; n<noOfNodes; n++)
		{
			Node tempNode = route.getRouteNodeList().get(n);
			Edge tempEdge = route.getRouteEdgeList().get(n);
			int depart = route.getDepartureTime().get(n);
			int edgeCapacity = Integer.MAX_VALUE;
			int edgeTravelTime = (int)Math.ceil(tempEdge.getTravelTime());
			int nodeCapacity = Integer.MAX_VALUE;
			boolean sameFlowDirection = false;
			if(tempNode==tempEdge.getSource())
				sameFlowDirection = true;
			
			//For eg : Departure time at 15, travel time = 5, Hence check capacity booked at  
			//slot 'time' for time 15 + time(0,1,2,3,4) = (15,16,17,18,19)
			int newCapacity = 0;
			if(sameFlowDirection)
			{
				newCapacity = tempEdge.getTempEdgeCapacity().get(depart);
			}
			else
			{
				newCapacity = tempEdge.getTempEdgeCapacity().get(depart + edgeTravelTime);
			}
			
			if(newCapacity < edgeCapacity)
			{
				edgeCapacity = newCapacity;
			}
			if(n != (noOfNodes - 1))
			{
				tempNode = route.getRouteNodeList().get(n+1);
				int arrival = (int)Math.ceil(tempNode.getTravelTime());
				nodeCapacity = tempNode.getNodeCapacityAtTime().get(arrival);
			}
			if(edgeCapacity < minCapacity && edgeCapacity <= nodeCapacity)
			{
				minCapacity = edgeCapacity;
			}
			else if(nodeCapacity < minCapacity && nodeCapacity < edgeCapacity)
			{
				minCapacity = nodeCapacity;
			}
		}
		if(route.getRouteNodeList().get(0).getCurrentOccupancy() < minCapacity)
			minCapacity = route.getRouteNodeList().get(0).getCurrentOccupancy();
		route.setGroupSize(minCapacity);
		return minCapacity;
	}
	
	public void shortestPathAlongRoute(Route route)
	{
		Route newRoute = new Route();
		newRoute.getArrivalTime().add(0);
		int capacity = route.getRouteNodeList().get(0).getCurrentOccupancy();
		
		for(int t=0;t<route.getRouteEdgeList().size();t++)
		{
			Node start = route.getRouteNodeList().get(t);
			Node next = route.getRouteNodeList().get(t+1);
			Edge edge = route.getRouteEdgeList().get(t);
			double departureTimeFromU=0;
			
			if(t!=0)
			{
				if(edge.getSource() == start)
					departureTimeFromU = start.getTravelTime();
				else
					departureTimeFromU = start.getTravelTime() + edge.getTravelTime(); 
					/*since we will reach source after edge travel time , it should not be filled at that time
					
					*  S------5--------T
					*  Here if we are going to move from T to S , we are going to reach S after 5 secs
					*  Hence the block is also happening after 5 secs
					*  It is not possible that the same edge is used in both directions
					*  We are travelling from start to next but we will always imitate 
					*  that we are going from next to start
					*/
			}
			else
			{
				if(edge.getSource() == start)
					departureTimeFromU = 0;
				else
					departureTimeFromU = edge.getTravelTime();
			}
			
			double timeInstancesToAdd = departureTimeFromU - edge.getTempEdgeCapacity().size() + 2;
		
			for(int i = 0; i < timeInstancesToAdd; i++)
				edge.addTempEdgeCapacity();
			
			//while the capacity in first section of this edge is not available
			//we will wait here only
			int delay = 0;
			while(edge.getTempEdgeCapacity().get((int)Math.ceil(departureTimeFromU)) <= 0)
			{
				delay++;
				departureTimeFromU++;
				if (edge.getTempEdgeCapacity().size() <= departureTimeFromU)
					edge.addTempEdgeCapacity();
			}

				//Adding Time instances for u
			timeInstancesToAdd = start.getTravelTime() + delay - 
					start.getTempNodeCapacityAtTime().size() + 1;
			for(int i = 0; i < timeInstancesToAdd; i++)
				start.getTempNodeCapacityAtTime().add(start.getMaxCapacity()-start.getCurrentOccupancy());
			
			double distanceToVThroughU = start.getTravelTime() + delay + edge.getTravelTime();

			/* Time instances to add to far vertex v
			 * We add at least that many time instances as the travel time 
			 * to node v
			*/
			timeInstancesToAdd = distanceToVThroughU - next.getTempNodeCapacityAtTime().size() + 2;

			for(int i = 0; i < timeInstancesToAdd; i++)
				next.getTempNodeCapacityAtTime().add(next.getMaxCapacity()-next.getCurrentOccupancy());

			//Capacity should be available at both the edge and at the vertex 
			while((next.getTempNodeCapacityAtTime().get((int)Math.ceil(distanceToVThroughU)) <= 0)
					|| edge.getTempEdgeCapacity().get((int)Math.ceil(departureTimeFromU)) <= 0)
			{
				delay++;
				distanceToVThroughU++;
				departureTimeFromU++;
				//Add time instance to node v
				next.getTempNodeCapacityAtTime().add(next.getMaxCapacity()-next.getCurrentOccupancy());
				start.getTempNodeCapacityAtTime().add(start.getMaxCapacity()-start.getCurrentOccupancy());
				//System.out.println(v.getNodeName() + "--" + v.getMaxCapacity());
				//Add time instances to edge uv
				while(edge.getTempEdgeCapacity().size() <=
						(departureTimeFromU + 1))
				{
					edge.addTempEdgeCapacity();
				}
			}
			next.setTravelTime(distanceToVThroughU);
			newRoute.getRouteEdgeList().add(edge);
			newRoute.getRouteNodeList().add(start);
			
			int depart = (int)Math.ceil(departureTimeFromU);
			int arrivalAtNext = (int)Math.ceil(distanceToVThroughU);
			newRoute.getDepartureTime().add(depart);
			newRoute.getArrivalTime().add(arrivalAtNext);
			
			int newCapacity = 0;
			if(edge.getSource() == start)
			{
				newCapacity = edge.getTempEdgeCapacity().get(depart);
			}
			else
			{
				newCapacity = edge.getTempEdgeCapacity().get(depart + (int)edge.getTravelTime());
			}
			if(newCapacity < capacity)
				capacity = newCapacity;
			
			newCapacity = next.getTempNodeCapacityAtTime().get(arrivalAtNext);
			if(newCapacity < capacity)
				capacity = newCapacity;				
		}
		
		
	}
	
	
	public Route shortestPath(Node src)
	{
		//System.out.println(src.getNodeName());
		PriorityQueue<Node> priorityQueue = new PriorityQueue<Node>();
		priorityQueue.add(src);
		Node u = src;
		double shortestEmptyTime = Double.MAX_VALUE;
		Route shortestRoute = null;
		while(u.getTravelTime() < shortestEmptyTime)
		{
			u = priorityQueue.poll();
			
			//System.out.println(u.getTravelTime() + "---" + u.getNodeName());
			if(u.getNodeType() == Node.DESTINATION)
			{
				//System.out.println(u.getTravelTime() + "---" + u.getNodeName());
				//System.out.println("A3");
				double firstTime = u.getTravelTime();
				Route r = findRoute(u);
				int delay = 0;
				int srcCap = src.getCurrentOccupancy();
				
				while(srcCap > 0)
				{
					int minCap = findMinimumCapacityAlongRoute(r, delay);
					r.addCapAtDelay(minCap);
					srcCap-=minCap;
					delay++;
				}
				delay--;
				double totalTime = firstTime + delay;
				//System.out.println("Total Time : " + totalTime);
				if(totalTime < shortestEmptyTime)
				{
					shortestEmptyTime = totalTime;
					shortestRoute = r;
				}
				
			}
			else
			{
				//for all adjacent vertices of u
				if(u.getEdgeTaken()!=null)
				{
					Edge edge = u.getEdgeTaken();
					Node v;
					if(edge.getSource() == u)
						v = edge.getTarget();
					else
						v = edge.getSource();
					simpleDijkstra(u, v, edge, priorityQueue);
					u.setScanned(true);
				}
				else
				{
					for(int index = 0;index<u.getAdjacencies().size();index++)
					{
						Node v;
						Edge edge = u.getAdjacencies().get(index);
						if(edge.getSource() == u)
							v = edge.getTarget();
						else
							v = edge.getSource();
						
						//System.out.println("---" + v.getNodeName());
						simpleDijkstra(u, v, edge, priorityQueue);
						u.setScanned(true);
					}
				}
			}
			if(priorityQueue.isEmpty())
				break;
			//System.out.println();
		}
		//got a destination node, trace the path back up to form the route.
		//src.setTimeToDestination(shortestEmptyTime);
		shortestRoute.setRouteTravelTime(shortestEmptyTime);
		return shortestRoute;
	}
	
	public void simpleDijkstra(Node u, Node v, Edge edge, PriorityQueue<Node> priorityQueue)
	{
		double departureTimeFromU;
		if(edge.getSource() == u)
			departureTimeFromU = u.getTravelTime();
		else
			departureTimeFromU = u.getTravelTime() + edge.getTravelTime(); //means we will reach source after travel time
		/* Time instances to add to an edge
		 * We add at least that many time instances as the travel time 
		 * of node u
		*/

		double timeInstancesToAdd = departureTimeFromU - edge.getEdgeCapacity().size() + 2;

		
		for(int i = 0; i < timeInstancesToAdd; i++)
			edge.addEdgeCapacity();
		
		//while the capacity in first section of this edge is not available
		//we will wait here only
		int delay = 0;
		while(edge.getEdgeCapacity().get((int)Math.ceil(departureTimeFromU)) <= 0)
		{
			delay++;
			departureTimeFromU++;
			if (edge.getEdgeCapacity().size() <= departureTimeFromU)
				edge.addEdgeCapacity();
		}

		//Adding Time instances for u
		timeInstancesToAdd = u.getTravelTime() + delay - 
				u.getNodeCapacityAtTime().size() + 1;
		for(int i = 0; i < timeInstancesToAdd; i++)
			u.getNodeCapacityAtTime().add(u.getMaxCapacity());
		
		double distanceToVThroughU = u.getTravelTime() + delay + edge.getTravelTime();

		/* Time instances to add to far vertex v
		 * We add at least that many time instances as the travel time 
		 * to node v
		*/
		timeInstancesToAdd = distanceToVThroughU - v.getNodeCapacityAtTime().size() + 2;

		for(int i = 0; i < timeInstancesToAdd; i++)
			v.getNodeCapacityAtTime().add(v.getMaxCapacity());

		//Capacity should be available at both the edge and at the vertex 
		while((v.getNodeCapacityAtTime().get((int)Math.ceil(distanceToVThroughU)) <= 0)
				|| edge.getEdgeCapacity().get((int)Math.ceil(departureTimeFromU)) <= 0)
		{
			delay++;
			distanceToVThroughU++;
			departureTimeFromU++;
			//Add time instance to node v
			v.getNodeCapacityAtTime().add(v.getMaxCapacity());
			u.getNodeCapacityAtTime().add(u.getMaxCapacity());
			//System.out.println(v.getNodeName() + "--" + v.getMaxCapacity());
			//Add time instances to edge uv
			while(edge.getEdgeCapacity().size() <=
					(departureTimeFromU + 1))
			{
				edge.addEdgeCapacity();
			}
			
			
		}

		if(distanceToVThroughU < v.getTravelTime())
		{

			if(v.getParent() != null)
			{
				//v has Parent
				v.getParent().removeChild(v);
			}
			v.setParent(u);
			u.addChild(v);

			PathUptoNode pathUptoPreviousNode = null;
			if(u.getPathUptoPreviousNode() == null)
			{
				//u is source node
				pathUptoPreviousNode = new PathUptoNode();
			}
			else
			{
				pathUptoPreviousNode = new PathUptoNode(u.getPathUptoPreviousNode());
			}
			pathUptoPreviousNode.add(u, edge, (int)Math.ceil(u.getTravelTime() + delay));
			v.setPathUptoPreviousNode(pathUptoPreviousNode);
			v.setTravelTime(distanceToVThroughU);
			if(priorityQueue.contains(v))
				priorityQueue.remove(v);
			priorityQueue.add(v);
			/* Time instances to add to an edge
			 * We add at least that many time instances as the travel time 
			 * of node v at end
			*/
			timeInstancesToAdd = distanceToVThroughU - 
					edge.getEdgeCapacity().size() + 2;

			for(int i = 0; i < timeInstancesToAdd; i++)
				edge.addEdgeCapacity();
		}
	}
	
	public Route findRoute(Node destination)
	{
		PathUptoNode pathUptoNode = destination.getPathUptoPreviousNode();
		
		Route route = new Route();
		route.setRouteId(pathId);
		route.getArrivalTime().add(0);	//Source node arrival time
		for(int index = 0; index < pathUptoNode.getNodeList().size(); index++)
		{
			Node temp = pathUptoNode.getNodeList().get(index);
			route.getNodeList().add(temp.getNodeName());
			route.getRouteNodeList().add(temp);
			if(index != 0)
				route.getArrivalTime().add((int)Math.ceil(temp.getTravelTime()));
			
			Edge tempEdge = pathUptoNode.getEdgeList().get(index);
			route.getEdgeList().add(tempEdge.getEdgeName());
			route.getRouteEdgeList().add(tempEdge);
			route.getDepartureTime().add(pathUptoNode.getDepartureTime().get(index));
		}
		route.getNodeList().add(destination.getNodeName());
		route.getRouteNodeList().add(destination);
		route.getArrivalTime().add((int)Math.ceil(destination.getTravelTime()));
		route.getDepartureTime().add(-1);  //Destination departure time
		route.setRouteTravelTime(destination.getTravelTime());
		return route;
	}
	
	public void displayNodeEdgeStats()
	{
		FileWriter writer = null;
		try
		{
		    writer = new FileWriter("/Users/MLGupta/Documents/PlusPlusNodeData101.csv");
		    writer.append("NodeName,NoOfPaths,NoOfPeople,WaitingTime\n");
			for(int i=0;i<graph.getNodeList().size();i++)
			{
				Node n = graph.getNodeList().get(i);
				String str = "";
				str+=n.getNodeName() + "," + n.getNoOfPathsThroughThisNode() + "," + n.getNoOfPeopleThroughThisNode() + "," + n.getWaitingTimeAtThisNode() + "\n";
				writer.write(str);
				//System.out.println(str);
			}
			writer.close();
		}
		catch(Exception e)
		{
			System.out.println(e.getMessage());
		}
		
		try
		{
		    writer = new FileWriter("/Users/MLGupta/Documents/PlusPlusRouteList101.csv");
		    writer.append("RouteList\n");
			for (Iterator<String> iterator = this.distinctRoutes.iterator(); iterator.hasNext();) {
				String route = (String) iterator.next();
				writer.write(route + "\n");
			}
			writer.close();
		}
		catch(Exception e)
		{
			System.out.println(e.getMessage());
		}
	}
	
}
