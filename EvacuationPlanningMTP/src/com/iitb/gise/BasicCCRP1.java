package com.iitb.gise;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.TreeMap;

public class BasicCCRP1 {

	private Graph graph = new Graph();
	private Set<Node> PQAdditionList = new HashSet<Node>();
	private static double humanWalkingSpeed = 1.5;			//in m/s
	private int pathId = 1;
	private ArrayList<Route> pathList = new ArrayList<Route>();
	private static Map<String,IntPair> distinctRoutes = new TreeMap<String,IntPair>();
	private int runNo = 0;
	
	
	public void clearData()
	{
		graph = null;
		graph = new Graph();
		PQAdditionList = null;
		PQAdditionList = new HashSet<Node>();
		pathId = 1;
		pathList = new ArrayList<Route>();
	}
	public int getRunNo() {
		return runNo;
	}

	public void setRunNo(int runNo) {
		this.runNo = runNo;
	}

	public Map<String,IntPair> getDistinctRoutes() {
		return distinctRoutes;
	}

	public void setDistinctRoutes(Map<String,IntPair> distinctRoutes) {
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
		BasicCCRP1.humanWalkingSpeed = humanWalkingSpeed;
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
			node.setTravelTime(0);
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
			node.setTravelTime(0);
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
	
	public void CCRPEvacuationPlanner()
	{
		Route.setTotalHops(0);
		PriorityQueue<Node> priorityQueue = new PriorityQueue<Node>();
		int evacueeCount = 0;
		int evacuationTime = 0;
		for(int index = 0;index<graph.getNodeList().size();index++)
		{
			Node node = graph.getNodeList().get(index);
			if(node.getNodeType() == Node.SOURCE)
			{
				//count people
				evacueeCount = evacueeCount + node.getInitialOccupancy();
			}
		}
		int tempcount = 0;
		//while there are people in any source node
		while(evacueeCount > 0)
		{
			priorityQueue.clear();
			//Adding all source nodes to priority queue
			for(int index = 0;index<graph.getNodeList().size();index++)
			{
				Node node = graph.getNodeList().get(index);
				if(node.getNodeType() == Node.SOURCE)
				{
					//Add source list to priority queue
					priorityQueue.add(node);
				}
			}
			
			Node u = priorityQueue.poll();
			
			//while u is not a destination
			while(u.getNodeType() != Node.DESTINATION)
			{
				//for all adjacent vertices of u
				for(int index = 0; index < u.getAdjacencies().size(); index++)
				{
					Edge edge = u.getAdjacencies().get(index);
					Node v;
					if(edge.getSource() == u)
						v = edge.getTarget();
					else
						v = edge.getSource();
					if (v.isScanned())
					{
						//Already visited , Do Nothing
					}
					else
					{
						simpleDijkstra(u, v, edge, priorityQueue);
					}
				}
				u.setScanned(true);
				u = priorityQueue.poll();
				
			}
			/*We have a destination node now with smallest distance*/
			evacuationTime += u.getTravelTime();
			//Find the minimum capacity cmin and the edge e where this bottleneck occurs
			Node source = u.getPathUptoPreviousNode().getNodeList().get(0);
			int sourceOccupancy = source.getCurrentOccupancy();
			MinimumCapacityAndEdge minCapacityAndEdge = findMinimumCapacityAndEdge(u);
			
			boolean sourceEmpty = false;
			int groupSize = 0;
			if(sourceOccupancy <= minCapacityAndEdge.getMinCapacity())
			{
				//Source should get empty
				sourceEmpty = true;
				groupSize = sourceOccupancy;
			}
			else
			{
		//		nodeToReset = minCapacityAndEdge.getNodeToReset();
				groupSize = minCapacityAndEdge.getMinCapacity();
			}
			//Reserve the path and assign the route to a group of size
			//min(cmin, evacuees in source of path found) from the source
			reservePath(u, groupSize);
			tempcount = tempcount + groupSize;
			//System.out.println(tempcount);
			//if(tempcount>32535)
				//break;
			if(sourceEmpty)
			{
				if(evacueeCount != groupSize)
				{
					//resetNodes(source, priorityQueue);
					//Source node becomes a normal node after getting empty
					source.setNodeType(Node.NORMAL);
					graph.clearNodeData();
				}
				else
				{
					//Evacuee count is equal to the group size means this is the last group.
					//Hence, no need to reset since the tree wont be used anymore.
					break;
				}
			}
			else
			{
				graph.clearNodeData();
			}

			evacueeCount = evacueeCount - groupSize;
		}
		//System.out.println(tempcount);
		System.out.println("Egress Time : " + pathList.get(pathList.size()-1).getArrivalTime().get(pathList.get(pathList.size()-1).getNodeList().size()-1));
		System.out.println("RouteList Size : " + pathList.size());
		System.out.println("Average Evacuation Time : " + 1.0*evacuationTime/tempcount);
		System.out.println("Avg Hops : " + 1.0*Route.getTotalHops()/pathList.size());
		System.out.println("Max Hops : " + Route.getMaxHops());
		System.out.println("No of Distinct Routes : " + this.getDistinctRoutes().size());
		int maxWaitingTimeAtANode = 0;
		String nodeName = "";
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
		System.out.println("Max. Waiting Time at a node : " + maxWaitingTimeAtANode 
				+ ", Node Name : " + nodeName);
		System.out.println("Average. Waiting Time at a node : " 
				+ totalWaitingTime/graph.getNodeList().size());
	}
	
	/*
	 * This will first reserve path from second node upto last edge
	 * Then it will increase capacity of source at departure time from the source node
	 * Then it will decrease occupancy of source node
	 * Add route to pathList
	 * Display the route
	 */
	public void reservePath(Node destination, int groupSize)
	{
		PathUptoNode pathUptoNode = destination.getPathUptoPreviousNode();
		int noOfNodes = pathUptoNode.getNodeList().size();
		for(int n=0; n<noOfNodes; n++)
		{
			Node tempNode = pathUptoNode.getNodeList().get(n);
			Edge tempEdge = pathUptoNode.getEdgeList().get(n);
			
			int depart = pathUptoNode.getDepartureTime().get(n);
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
			
			Node currNode = pathUptoNode.getNodeList().get(n);
			currNode.setNoOfPathsThroughThisNode(currNode.getNoOfPathsThroughThisNode() + 1);
			currNode.setNoOfPeopleThroughThisNode(currNode.getNoOfPeopleThroughThisNode() + groupSize);
			
			int currNodeArrival = 0;
			if(n!=0)
				currNodeArrival = (int)currNode.getTravelTime();
			
			int departure = pathUptoNode.getDepartureTime().get(n);
			currNode.setWaitingTimeAtThisNode(departure-currNodeArrival);
			
			for(int i=currNodeArrival;i<departure;i++)
			{
				int newNodeCapacity = tempNode.getNodeCapacityAtTime().get(i) - groupSize;
				tempNode.getNodeCapacityAtTime().set(i, newNodeCapacity);
			}
		}
		
		Node src = destination.getPathUptoPreviousNode().getNodeList().get(0);
		src.setCurrentOccupancy(src.getCurrentOccupancy() - groupSize);
		
		//Add path to pathList
		Route route = new Route();
		route.setRouteId(pathId);
		route.setGroupSize(groupSize);
		route.getArrivalTime().add(0);	//Source node arrival time
		for(int index = 0; index < pathUptoNode.getNodeList().size(); index++)
		{
			Node temp = pathUptoNode.getNodeList().get(index);
			route.getNodeList().add(temp.getNodeName());
			if(index != 0)
				route.getArrivalTime().add((int)Math.ceil(temp.getTravelTime()));
			
			Edge tempEdge = pathUptoNode.getEdgeList().get(index);
			route.getEdgeList().add(tempEdge.getEdgeName());
			
			route.getDepartureTime().add(pathUptoNode.getDepartureTime().get(index));
		}
		route.getNodeList().add(destination.getNodeName());
		route.getArrivalTime().add((int)Math.ceil(destination.getTravelTime()));
		route.getDepartureTime().add(-1);  //Destination departure time
		this.pathList.add(route);
		
		Route.addTotalHops(route.getNodeList().size());
		if(Route.getMaxHops() < route.getNodeList().size())
			Route.setMaxHops(route.getNodeList().size());
		
		if(destination.getTravelTime() > Route.getMaxLength())
			Route.setMaxLength(destination.getTravelTime());
		
		//route.displayRoute2();
		String str1 = route.getRouteString();
		if(distinctRoutes.containsKey(str1))
		{
			if(this.getRunNo()==0)
			{
				IntPair intPair = distinctRoutes.get(str1);
				int a= intPair.getA1();
				a= a+ route.getGroupSize();
				intPair.setA1(a);
				distinctRoutes.put(str1, intPair);
			}
			else
			{
				IntPair intPair = distinctRoutes.get(str1);
				int a= intPair.getA2();
				a= a+ route.getGroupSize();
				intPair.setA2(a);
				distinctRoutes.put(str1, intPair);
			}
		}
		else
		{
			IntPair intPair = new IntPair();
			if(this.getRunNo()==0)
				intPair.setA1(route.getGroupSize());
			else
				intPair.setA2(route.getGroupSize());
			distinctRoutes.put(str1,intPair);
		}
		this.pathId++;
	}
	public MinimumCapacityAndEdge findMinimumCapacityAndEdge(Node destination)
	{
		PathUptoNode pathUptoNode = destination.getPathUptoPreviousNode();
		int noOfNodes = pathUptoNode.getNodeList().size();	
		
		int minCapacity = Integer.MAX_VALUE;
		Edge minCutEdge = null;
		Node nodeToReset = null;
		for(int n=0; n<noOfNodes; n++)
		{
			//Comparing minCapaciy, edge and nextNode
			Node tempNode = pathUptoNode.getNodeList().get(n);
			Edge tempEdge = pathUptoNode.getEdgeList().get(n);
			int depart = pathUptoNode.getDepartureTime().get(n);
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
				tempNode = pathUptoNode.getNodeList().get(n+1);
				int arrival = (int)Math.ceil(tempNode.getTravelTime());
				nodeCapacity = tempNode.getNodeCapacityAtTime().get(arrival);
			}
			if(edgeCapacity < minCapacity && edgeCapacity <= nodeCapacity)
			{
				minCapacity = edgeCapacity;
				/*minCutEdge = tempEdge;
				if(n == (noOfNodes - 1))
				{
					//nodeToReset is destinationNode
					nodeToReset = destination;
				}
				else
				{
					nodeToReset = tempNode;
				}*/
			}
			else if(nodeCapacity < minCapacity && nodeCapacity < edgeCapacity)
			{
				minCapacity = nodeCapacity;
			/*	minCutEdge = tempEdge;
				nodeToReset = tempNode;*/
			}
		}
		MinimumCapacityAndEdge minimumCapacityAndEdge = 
				new MinimumCapacityAndEdge(minCapacity, minCutEdge, nodeToReset);
		return minimumCapacityAndEdge;
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
	
	public void displayNodeEdgeStats()
	{
		FileWriter writer = null;
		try
		{
		    writer = new FileWriter("/Users/MLGupta/33N.csv");
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
		    writer = new FileWriter("/Users/MLGupta/R_33.csv");
		   // writer.append("RouteList\n");
			for (Iterator<Route> iterator = this.pathList.iterator(); iterator.hasNext();) {
				Route route = (Route) iterator.next();
				writer.write(route.getRouteString() + "," + route.getGroupSize() + "\n");
			}
			writer.close();
		}
		catch(Exception e)
		{
			System.out.println(e.getMessage());
		}
		
		try
		{
		    writer = new FileWriter("/Users/MLGupta/BothAlgoInfo3.csv");
		   // writer.append("RouteList\n");
		    Iterator it = distinctRoutes.entrySet().iterator();
		    while (it.hasNext()) {
		        Map.Entry<String,IntPair> pairs = (Map.Entry<String,IntPair>)it.next();
		        writer.write(pairs.getKey() + "," + pairs.getValue().getA1() + "," + pairs.getValue().getA2() + "\n");
		        it.remove(); // avoids a ConcurrentModificationException
		    }
			writer.close();
		}
		catch(Exception e)
		{
			System.out.println(e.getMessage());
		}
	}
}
