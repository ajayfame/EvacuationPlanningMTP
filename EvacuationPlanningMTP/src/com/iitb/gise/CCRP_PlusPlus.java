package com.iitb.gise;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

public class CCRP_PlusPlus {

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
		CCRP_PlusPlus.humanWalkingSpeed = humanWalkingSpeed;
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
		
		//For each section of edge, initially(t=0) capacity is its maximum
		for (int i = 0; i < travelTime; i++)
		{
			edgeCapacityAtThisTime.getCapacityOfSection().add(maxIntakeCapacity);
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
		
		//For each section of edge, initially(t=0) capacity is its maximum
		for (int i = 0; i < travelTime; i++)
		{
			edgeCapacityAtThisTime.getCapacityOfSection().add(maxIntakeCapacity);
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
		
		//For each section of edge, initially(t=0) capacity is its maximum
		for (int i = 0; i < travelTime; i++)
		{
			edgeCapacityAtThisTime.getCapacityOfSection().add(maxIntakeCapacity);
		}
		
		edge.addEdgeCapacityAtNextTimeInstance(edgeCapacityAtThisTime);
		EdgeCapacityAtThisTime edgeMaxCapacityTime = new EdgeCapacityAtThisTime(edgeCapacityAtThisTime);
		edge.setEdgeMaxCapacityTimeInstance(edgeMaxCapacityTime);
		graph.addEdge(edge);
		
		src.addAdjacentEdge(edge);
		//target.addAdjacentEdge(edge);
	}
	
	public void ccrpPlusPlus()
	{
		Route.setTotalHops(0);
		//For each source find the shortest available path to a destination.
		PriorityQueue<Route> preRQ = new PriorityQueue<Route>();
		PriorityQueue<Route> RQ = new PriorityQueue<Route>();
		
		int totalEvacuees = 0;
		for(int i=0;i<srcList.size();i++)
		{
			Node src = srcList.get(i);
			totalEvacuees += src.getInitialOccupancy();
			clearNodeData(src);
			Route r = shortestPath(src);
			preRQ.add(r);
		}
		RQ.add(preRQ.poll());
		int count = 0;
		while(count < totalEvacuees)
		{
			if(RQ.isEmpty() && !preRQ.isEmpty())
			{
				RQ.add(preRQ.poll());
			}
			else if(RQ.isEmpty() && preRQ.isEmpty())
			{
				System.out.println("Error");
				break;
			}
			Route q1 = RQ.peek();
			
			Node src1 = q1.getRouteNodeList().get(0);
			clearNodeData(src1);
			q1 = shortestPath(src1);
			RQ.poll();
			RQ.add(q1);
			Route p1 = preRQ.peek();
			
			if(p1 != null && q1!=null && p1.getRouteTravelTime() < q1.getRouteTravelTime())
			{
				Node src = p1.getRouteNodeList().get(0);
				clearNodeData(src);
				Route r = shortestPath(src);
				
				if(check(r,p1))
				{
					preRQ.poll();
					RQ.add(r);
				}
				else
				{
					preRQ.poll();
					preRQ.add(r);
				}
			}
			else
			{
				int minCap = findMinimumCapacity(q1);
				reservePath(q1, minCap);
				evacuationTime+= q1.getRouteTravelTime();
				//this.pathList.add(q1);
				count+=minCap;
				RQ.poll();
				if(q1.getRouteNodeList().get(0).getCurrentOccupancy()<=0)
				{
					continue;
				}
				if(RQ.isEmpty())
				{
					if(!preRQ.isEmpty())
					{
						RQ.add(preRQ.poll());
					}
					else
					{
						if(q1.getRouteNodeList().get(0).getCurrentOccupancy() > 0)
						{
							while(q1.getRouteNodeList().get(0).getCurrentOccupancy() > 0)
							{
								Node src = q1.getRouteNodeList().get(0);
								clearNodeData(src);
								q1 = shortestPath(src);
								minCap = findMinimumCapacity(q1);
								reservePath(q1, minCap);
								evacuationTime+= q1.getRouteTravelTime();
								//this.pathList.add(q1);
								//q1.displayRoute();
								count+=minCap;
							}
							continue;
						}
					}
				}
				Route q2 = RQ.peek();
				Node src = q1.getRouteNodeList().get(0);
				clearNodeData(src);
				q1 = shortestPath(src);
				while(q1.getRouteNodeList().get(0).getCurrentOccupancy() > 0 && 
						q1.getRouteTravelTime() <= q2.getRouteTravelTime())
				{
					minCap = findMinimumCapacity(q1);
					reservePath(q1, minCap);
					evacuationTime+= q1.getRouteTravelTime();
					//this.pathList.add(q1);
					//q1.displayRoute();
					count+=minCap;
					src = q1.getRouteNodeList().get(0);
					clearNodeData(src);
					q1 = shortestPath(src);
				}
				if(q1.getRouteNodeList().get(0).getCurrentOccupancy() > 0)
				{
					src = q1.getRouteNodeList().get(0);
					clearNodeData(src);
					q1 = shortestPath(src);
					preRQ.add(q1);
				}
			}
		}
		System.out.println("Egress Time : " + pathList.get(pathList.size()-1).getArrivalTime().get(pathList.get(pathList.size()-1).getNodeList().size()-1));
		System.out.println("RouteList Size : " + pathList.size());
		System.out.println("Average Evacuation Time : " + 1.0*evacuationTime/count);
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
		System.out.println("Count:" + count);
	}
	
	public void reservePath(Route route, int groupSize)
	{
		//PathUptoNode pathUptoNode = destination.getPathUptoPreviousNode();
		int noOfNodes = route.getRouteNodeList().size()-1;
		for(int n=0; n<noOfNodes; n++)
		{
			Node tempNode;
			Edge tempEdge = route.getRouteEdgeList().get(n);
			
			int depart = route.getDepartureTime().get(n);
			int edgeTravelTime = (int)Math.ceil(tempEdge.getTravelTime());
			//Edge reservation
			for(int time=0; time < edgeTravelTime; time++)
			{
				int newCapacity = tempEdge.getEdgeCapacityAtTimeInstance().
						get(depart + time).getCapacityOfSection().get(time) - groupSize;
				//For eg : Departure time at 15, travel time = 5, Hence slot 'time' should be 
				//booked at time 15 + time(0,1,2,3,4) = (15,16,17,18,19)
				tempEdge.getEdgeCapacityAtTimeInstance().get(depart + time).getCapacityOfSection().set(
						time, newCapacity);
			}
			Node currNode = route.getRouteNodeList().get(n);
			currNode.setNoOfPathsThroughThisNode(currNode.getNoOfPathsThroughThisNode() + 1);
			currNode.setNoOfPeopleThroughThisNode(currNode.getNoOfPeopleThroughThisNode() + groupSize);
			int currNodeArrival = 0;
			if(n!=0)
				currNodeArrival = (int)currNode.getTravelTime();
			
			int departure = currNodeArrival;
			if(n!=noOfNodes-1)
				departure = route.getDepartureTime().get(n);
			currNode.setWaitingTimeAtThisNode(departure-currNodeArrival);
			
			if(n != (noOfNodes - 1))
			{
				//Not destination
				tempNode = route.getRouteNodeList().get(n+1);
				int arrival = (int)Math.ceil(route.getArrivalTime().get(n+1));
				int newNodeCapacity = tempNode.getNodeCapacityAtTime().get(arrival) - groupSize;
				
				tempNode.getNodeCapacityAtTime().set(arrival, newNodeCapacity);
			}
		}
		
		//Change the capacity of source
		//If departing from the source node at 15, capacity should be 
		//increased for time slot 15 onwards
		int depart = route.getDepartureTime().get(0);
		Node src = route.getRouteNodeList().get(0);

		//Check whether this time slot will be present or not in src node
		int newOccupancyAtDeparture = src.getNodeCapacityAtTime().get(depart) + groupSize;
		src.getNodeCapacityAtTime().set(depart, newOccupancyAtDeparture);
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
			//Comparing minCapaciy, edge and nextNode
			Node tempNode = null;
			Edge tempEdge = route.getRouteEdgeList().get(n);
			int depart = route.getDepartureTime().get(n);
			int edgeCapacity = Integer.MAX_VALUE;
			int edgeTravelTime = (int)Math.ceil(tempEdge.getTravelTime());
			int nodeCapacity = Integer.MAX_VALUE;
			for(int time=0; time < edgeTravelTime; time++)
			{
				//For eg : Departure time at 15, travel time = 5, Hence check capacity booked at  
				//slot 'time' for time 15 + time(0,1,2,3,4) = (15,16,17,18,19)
				int newCapacity = tempEdge.getEdgeCapacityAtTimeInstance().
						get(depart + time).getCapacityOfSection().get(time);
				if(newCapacity < edgeCapacity)
				{
					edgeCapacity = newCapacity;
				}
			}
			if(n != (noOfNodes - 1))
			{
				tempNode = route.getRouteNodeList().get(n+1);
				int arrival = route.getArrivalTime().get(n+1);
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
	
	public Route shortestPath(Node src)
	{
		PriorityQueue<Node> priorityQueue = new PriorityQueue<Node>();
		priorityQueue.add(src);
		Node u = src;
		while(u.getNodeType() != Node.DESTINATION)
		{
			//for all adjacent vertices of u
				
			for(int index = 0;index<u.getAdjacencies().size();index++)
			{
				Node v;
				Edge edge = u.getAdjacencies().get(index);
				if(edge.getSource() == u)
					v = edge.getTarget();
				else
					v = edge.getSource();
				
				simpleDijkstra(u, v, edge, priorityQueue);
				u.setScanned(true);
			}
			u = priorityQueue.poll();
		}
		//got a destination node, trace the path back up to form the route.
		Route route = findRoute(u);
		return route;
		
	}
	
	public void simpleDijkstra(Node u, Node v, Edge edge, PriorityQueue<Node> priorityQueue)
	{
		double departureTimeFromU = u.getTravelTime();
		/* Time instances to add to an edge
		 * We add at least that many time instances as the travel time 
		 * of node u
		*/

		double timeInstancesToAdd = departureTimeFromU - 
				edge.getEdgeCapacityAtTimeInstance().size() + 2;
		EdgeCapacityAtThisTime edgeCapacityAtThisTime = 
				edge.getEdgeCapacityAtTimeInstance().get(
						edge.getEdgeCapacityAtTimeInstance().size() - 1);
		
		for(int i = 0; i < timeInstancesToAdd; i++)
			edge.addEdgeCapacityAtNextTimeInstance(edgeCapacityAtThisTime);
		
		//while the capacity in first section of this edge is not available
		//we will wait here only
		while(edge.getEdgeCapacityAtTimeInstance().get(
				(int)Math.ceil(departureTimeFromU)).
				getCapacityOfSection().get(0) <= 0)
		{
			departureTimeFromU++;
			if (edge.getEdgeCapacityAtTimeInstance().size() <= 
									(int)Math.ceil(departureTimeFromU))
				edge.addEdgeCapacityAtNextTimeInstance(
						edge.getEdgeMaxCapacityTimeInstance());
		}

		
		//Adding Time instances for u
		timeInstancesToAdd = departureTimeFromU - 
				u.getNodeCapacityAtTime().size() + 2;
		int lastNodeCapacity = u.getNodeCapacityAtTime().get(
						u.getNodeCapacityAtTime().size() - 1);
		for(int i = 0; i < timeInstancesToAdd; i++)
			u.getNodeCapacityAtTime().add(lastNodeCapacity);
		
		double edgeTravelTime = (int)Math.ceil(edge.getTravelTime());
		double distanceToVThroughU = departureTimeFromU + edgeTravelTime;

		/* Time instances to add to far vertex v
		 * We add at least that many time instances as the travel time 
		 * to node v
		*/
		timeInstancesToAdd = distanceToVThroughU - 
				v.getNodeCapacityAtTime().size() + 2;
		lastNodeCapacity = v.getNodeCapacityAtTime().get(
						v.getNodeCapacityAtTime().size() - 1);
		for(int i = 0; i < timeInstancesToAdd; i++)
			v.getNodeCapacityAtTime().add(lastNodeCapacity);

		//Capacity should be available at both the edge and at the vertex 
		while((v.getNodeCapacityAtTime().get((int)Math.ceil(distanceToVThroughU)) 
									<= 0)
				|| edge.getEdgeCapacityAtTimeInstance().get(
						(int)Math.ceil(departureTimeFromU)).
						getCapacityOfSection().get(0) <= 0
				)
		{
			distanceToVThroughU++;
			departureTimeFromU++;
			//Add time instance to node v
			v.getNodeCapacityAtTime().add(v.getMaxCapacity());
			u.getNodeCapacityAtTime().add(u.getMaxCapacity());
			//Add time instances to edge uv
			while(edge.getEdgeCapacityAtTimeInstance().size() <=
					((int)Math.ceil(departureTimeFromU) + 1))
			{
				edge.addEdgeCapacityAtNextTimeInstance(
						edge.getEdgeMaxCapacityTimeInstance());
			}
		}

		if(distanceToVThroughU < v.getTravelTime())
		{
			//System.out.println("abc");
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
			pathUptoPreviousNode.add(u, edge, (int)Math.ceil(departureTimeFromU));
			v.setPathUptoPreviousNode(pathUptoPreviousNode);
			v.setTravelTime(distanceToVThroughU);
			priorityQueue.add(v);
			
			/* Time instances to add to an edge
			 * We add at least that many time instances as the travel time 
			 * of node v at end
			*/
			timeInstancesToAdd = distanceToVThroughU - 
					edge.getEdgeCapacityAtTimeInstance().size() + 2;
			edgeCapacityAtThisTime = 
					edge.getEdgeCapacityAtTimeInstance().get(
							edge.getEdgeCapacityAtTimeInstance().size() - 1);
			for(int i = 0; i < timeInstancesToAdd; i++)
				edge.addEdgeCapacityAtNextTimeInstance(edgeCapacityAtThisTime);
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
}
