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

public class PreferenceToLongerPaths {

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
		PreferenceToLongerPaths.humanWalkingSpeed = humanWalkingSpeed;
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
		{
			node.setInitialOccupancy(0);
			node.setCurrentOccupancy(0);
			node.setMaxCapacity(Integer.MAX_VALUE);
		}
		if(nodeType == Node.SOURCE)
		{
			node.setTravelTime(0);
			srcList.add(node);
		}
		else
			node.setTravelTime(Double.MAX_VALUE);
		ArrayList<Integer> nodeCapacityAtTime = new ArrayList<Integer>();
		nodeCapacityAtTime.add(node.getMaxCapacity()-node.getInitialOccupancy());
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
		{
			node.setInitialOccupancy(0);
			node.setCurrentOccupancy(0);
			node.setMaxCapacity(Integer.MAX_VALUE);
		}
		if(nodeType == Node.SOURCE)
		{
			node.setTravelTime(0);
			srcList.add(node);
		}
		else
			node.setTravelTime(Double.MAX_VALUE);
		ArrayList<Integer> nodeCapacityAtTime = new ArrayList<Integer>();
		nodeCapacityAtTime.add(node.getMaxCapacity() - node.getInitialOccupancy());
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
	
	public void ccrpPlusPlus()
	{
		Route.setTotalHops(0);
		//For each source find the shortest available path to a destination.
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
		PriorityQueue<Route> preRQ = new PriorityQueue<Route>(srcList.size(), comp);
		PriorityQueue<Route> RQ = new PriorityQueue<Route>(srcList.size(), comp);
		
		int totalEvacuees = 0;
		for(int i=0;i<srcList.size();i++)
		{
			Node src = srcList.get(i);
			totalEvacuees += src.getInitialOccupancy();
			clearNodeData(src);
			//System.out.println("aaa");
			Route r = shortestPath(src);
			//System.out.println("aaa");
			//System.out.println(r.getRouteString());
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
			
			if(p1 != null && q1!=null && p1.getRouteTravelTime() > q1.getRouteTravelTime())
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
						q1.getRouteTravelTime() >= q2.getRouteTravelTime())
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
		System.out.println("Egress Time : " + Route.getMaxLength());
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
				newCapacity = tempEdge.getEdgeCapacity().get(depart) - groupSize;
				tempEdge.getEdgeCapacity().set(depart, newCapacity);
			}
			else
			{
				newCapacity = tempEdge.getEdgeCapacity().get(depart + edgeTravelTime) - groupSize;
				tempEdge.getEdgeCapacity().set(depart + edgeTravelTime, newCapacity);
			}
			//For eg : Departure time at 15, travel time = 5, Hence slot 'time' should be 
			//booked at time 15 + time(0,1,2,3,4) = (15,16,17,18,19)
			
			Node currNode = route.getRouteNodeList().get(n);
			currNode.setNoOfPathsThroughThisNode(currNode.getNoOfPathsThroughThisNode() + 1);
			currNode.setNoOfPeopleThroughThisNode(currNode.getNoOfPeopleThroughThisNode() + groupSize);
			
			int currNodeArrival = 0;
			if(n!=0)
				currNodeArrival = (int)currNode.getTravelTime();
			
			int departure = route.getDepartureTime().get(n);
			currNode.setWaitingTimeAtThisNode(departure-currNodeArrival);
			
			for(int i=currNodeArrival;i<departure;i++)
			{
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
		
		if(Route.getMaxLength() < route.getRouteTravelTime())
			Route.setMaxLength(route.getRouteTravelTime());
		distinctRoutes.add(route.getRouteString());
		//System.out.println(route.getRouteString());
		this.pathId++;
	}
	
	public int findMinimumCapacity(Route route)
	{
		int noOfNodes = route.getRouteNodeList().size()-1;
		int minCapacity = route.getRouteNodeList().get(0).getCurrentOccupancy();
		//without destination
		for(int n=0; n<noOfNodes; n++)
		{
			Node tempNode = route.getRouteNodeList().get(n);
			if(tempNode.getNodeName().equals("n483"))
				System.out.println("gggg");
			
			Edge tempEdge = route.getRouteEdgeList().get(n);
			int sourceArrival = route.getArrivalTime().get(n);
			int depart = route.getDepartureTime().get(n);
			int minSourceCap = Integer.MAX_VALUE;
			for(int i=sourceArrival;i<=depart;i++)
			{
				int v = tempNode.getNodeCapacityAtTime().get(i);
				if(minSourceCap > v)
				{
					minSourceCap = v;
				}
			}
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
				int arrival = route.getArrivalTime().get(n+1);
				nodeCapacity = tempNode.getNodeCapacityAtTime().get(arrival);
			}
			if(minSourceCap < minCapacity)
			{
				minCapacity = minSourceCap;
			}
			if(edgeCapacity < minCapacity)
			{
				minCapacity = edgeCapacity;
			}
			else if(nodeCapacity < minCapacity)
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
				//System.out.println("ffff");
				simpleDijkstra(u, v, edge, priorityQueue);
				//System.out.println("gggg");

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
		double departureTimeFromU;
		if(edge.getSource() == u)
			departureTimeFromU = u.getTravelTime();
		else
			departureTimeFromU = u.getTravelTime() + edge.getTravelTime(); //means we will reach source after travel time
		/* Time instances to add to an edge
		 * We add at least that many time instances as the travel time 
		 * of node u
		*/
		
		if(u.getNodeName().equals("n483") || v.getNodeName().equals("n483") )
			System.out.print("");

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

		//System.out.println("gggg");
		//Adding Time instances for u
		timeInstancesToAdd = u.getTravelTime() + delay - 
				u.getNodeCapacityAtTime().size() + 1;
		
			
		for(int i = 0; i < timeInstancesToAdd; i++)
			u.getNodeCapacityAtTime().add(u.getMaxCapacity()-u.getCurrentOccupancy());
		
		double distanceToVThroughU = u.getTravelTime() + delay + edge.getTravelTime();

		/* Time instances to add to far vertex v
		 * We add at least that many time instances as the travel time 
		 * to node v
		*/
		timeInstancesToAdd = distanceToVThroughU - v.getNodeCapacityAtTime().size() + 2;

		//System.out.println("hhhh");
		
		for(int i = 0; i < timeInstancesToAdd; i++)
			v.getNodeCapacityAtTime().add(v.getMaxCapacity()-v.getCurrentOccupancy());

		//Capacity should be available at both the edge and at the vertex 
		while((v.getNodeCapacityAtTime().get((int)Math.ceil(distanceToVThroughU)) <= 0)
				|| edge.getEdgeCapacity().get((int)Math.ceil(departureTimeFromU)) <= 0)
		{
			delay++;
			distanceToVThroughU++;
			departureTimeFromU++;
			
			
			//Add time instance to node v
			v.getNodeCapacityAtTime().add(v.getMaxCapacity()-v.getCurrentOccupancy());
			u.getNodeCapacityAtTime().add(u.getMaxCapacity()-u.getCurrentOccupancy());
			//Add time instances to edge uv
			if(u.getNodeName().equals("n483") || v.getNodeName().equals("n483") )
				System.out.println("ffff");
			System.out.println(distanceToVThroughU + " " + v.getNodeCapacityAtTime().get((int)Math.ceil(distanceToVThroughU)));
			if(v.getNodeCapacityAtTime().get((int)Math.ceil(distanceToVThroughU)) < 0)
				System.out.println(v.getNodeName());
		 	while(edge.getEdgeCapacity().size() <=
					(departureTimeFromU + 1))
			{
				edge.addEdgeCapacity();
			}
		}

		//System.out.println("iiii");
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
		//System.out.println("jjjj ");
	}
	
	
	public void simpleDijkstra1(Node u, Node v, Edge edge, PriorityQueue<Node> priorityQueue)
	{
		//u is edge source and v is edge target
		
		int departureTimeFromU =(int)Math.ceil(u.getTravelTime());

		//We will check whether there is any flow coming from other side of edge. hence we add edge.getTravelTime()
		double timeInstancesToAdd = departureTimeFromU - edge.getEdgeCapacity().size() + edge.getTravelTime() + 2;
		
		for(int i = 0; i < timeInstancesToAdd; i++)
			edge.addEdgeCapacity();
		
		//while the capacity in first section of this edge is not available
		//we will wait here only
		int delay = 0;
		boolean x = true, y=true;
		while(true)
		{
			x = false; 
			while(edge.getEdgeCapacity().get(departureTimeFromU) <= 0)
			{
				x = true;
				delay++;
				departureTimeFromU++;
				if (edge.getEdgeCapacity().size() <= (departureTimeFromU + edge.getTravelTime()))
					edge.addEdgeCapacity();
			}
			int i=departureTimeFromU + 1 - (int)edge.getTravelTime();
			if(i<0)
				i=0;
			
			y=false;
			for(; i < (departureTimeFromU + edge.getTravelTime()); i++)
			{
				if(edge.getEdgeCapacity().get(i) < edge.getMaxIntakeCapacity())
				{
					//There is flow in opposite direction
					y=true;
					delay++;
					departureTimeFromU++;
					if (edge.getEdgeCapacity().size() <= (departureTimeFromU + edge.getTravelTime()))
						edge.addEdgeCapacity();
				}
			}
			if(!x && !y)
				break;
		}
		//System.out.println("gggg");
		//Adding Time instances for u
		timeInstancesToAdd = u.getTravelTime() + delay - 
				u.getNodeCapacityAtTime().size() + 1;
		
			
		for(int i = 0; i < timeInstancesToAdd; i++)
			u.getNodeCapacityAtTime().add(u.getMaxCapacity()-u.getCurrentOccupancy());
		
		double distanceToVThroughU = u.getTravelTime() + delay + edge.getTravelTime();

		/* Time instances to add to far vertex v
		 * We add at least that many time instances as the travel time 
		 * to node v
		*/
		timeInstancesToAdd = distanceToVThroughU - v.getNodeCapacityAtTime().size() + 2;

		//System.out.println("hhhh");
		
		for(int i = 0; i < timeInstancesToAdd; i++)
			v.getNodeCapacityAtTime().add(v.getMaxCapacity()-v.getCurrentOccupancy());

		//I will reach u only if I have capacity, hence no need to check u
		//Capacity should be available at both the edge and at the vertex 
		while((v.getNodeCapacityAtTime().get((int)Math.ceil(distanceToVThroughU)) <= 0)
				|| edge.getEdgeCapacity().get((int)Math.ceil(departureTimeFromU)) <= 0)
		{
			delay++;
			distanceToVThroughU++;
			departureTimeFromU++;
			
			
			//Add time instance to node v
			v.getNodeCapacityAtTime().add(v.getMaxCapacity()-v.getCurrentOccupancy());
			u.getNodeCapacityAtTime().add(u.getMaxCapacity()-u.getCurrentOccupancy());
			//Add time instances to edge uv
			if(u.getNodeName().equals("n483") || v.getNodeName().equals("n483") )
				System.out.println("ffff");
			System.out.println(distanceToVThroughU + " " + v.getNodeCapacityAtTime().get((int)Math.ceil(distanceToVThroughU)));
			if(v.getNodeCapacityAtTime().get((int)Math.ceil(distanceToVThroughU)) < 0)
				System.out.println(v.getNodeName());
		 	while(edge.getEdgeCapacity().size() <=
					(departureTimeFromU + 1))
			{
				edge.addEdgeCapacity();
			}
		}

		//System.out.println("iiii");
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
		//System.out.println("jjjj ");
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
