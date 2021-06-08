package frc.robot;

import java.util.List;
import java.util.ArrayList;
import java.util.PriorityQueue;
import java.io.FileOutputStream;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.w3c.dom.Element;
import org.w3c.dom.NamedNodeMap;
import java.util.Map;
import java.util.HashMap;

/**
 * Contains all of the waypoint information used to navigate during autonomous. 
 * A
 * ll positions are in meters, relative to the back left corner of the playable field area from the 
 * point of view of the robot.
 * This class does not contain all of the functionality that it does in the
 * A Star Prototype project.
*/
class WaypointLattice
{
    public List<Waypoint> waypoints = new ArrayList<Waypoint>();
    /**
     * Use this to access the waypoints & locations for various parts of the field.
     * The exact string names for each 
     */
    public Map<String, Waypoint> targets = new HashMap<String, Waypoint>();

    public synchronized Waypoint GetNearestWaypoint(Vector2 pos)
    {
        Waypoint nearest = null;
        double minDist = Double.MAX_VALUE;
        for (Waypoint w : waypoints)
        {
            //Euclidean distance w/o square root
            //Because we only need to compare the distances and square roots are expensive
            double dist = Math.pow(w.position.x - pos.x, 2.0) + Math.pow(w.position.y - pos.y, 2.0);
            if (dist < minDist)
            {
                nearest = w;
                minDist = dist;
            }
        }
        return nearest;
    }
    public synchronized Waypoint GetNearestWaypointWithinDistance(Vector2 pos, double distance)
    {
        Waypoint nearest = null;
        double minDist = Double.MAX_VALUE;
        for (Waypoint w : waypoints)
        {
            double dist = pos.subtractedBy(w.position).length();
            if (dist < minDist && dist < distance)
            {
                nearest = w;
                minDist = dist;
            }
        }
        return nearest;
    }
    public synchronized ArrayList<Waypoint> GetWaypointsInRect(Vector2 min, Vector2 max)
    {
        ArrayList<Waypoint> intersecting = new ArrayList<Waypoint>();
        for (Waypoint waypoint : waypoints)
        {
            if (waypoint.position.x > min.x && waypoint.position.x < max.x 
                && waypoint.position.y > min.y && waypoint.position.y < max.y)
            {
                intersecting.add(waypoint);
            }
        }
        return intersecting;
    }
    /**Removes a waypoint and severs all of its connections */
    public synchronized void RemoveWaypoint(Waypoint removed)
    {
        for (Waypoint neighbor : removed.neighbors)
        {
            neighbor.neighbors.remove(removed);
        }
        waypoints.remove(removed);
    }
    /**
     * Uses the A* pathfinding algorithm to find the shortest path between "start" and "end" 
     * I don't have the time or the room to explain it; just look it up.
     * @param turnResistance Increase this number to have the path avoid sharp turns. Recommended value: 100.
    */
    public synchronized ArrayList<Waypoint> CalculatePath(Waypoint start, Waypoint end, double turnResistance)
    {
        if (start == null || end == null) return null;
        //Search the grid
        PriorityQueue<Waypoint> frontier = new PriorityQueue<Waypoint>();
        ArrayList<Waypoint> explored = new ArrayList<Waypoint>();
        start.Reset();
        start.onFrontier = true;
        frontier.add(start);
        Waypoint waypoint = null;
        while (!frontier.isEmpty())
        {
            waypoint = frontier.poll();
            waypoint.onFrontier = false;
            waypoint.explored = true;
            explored.add(waypoint);
            if (waypoint == end) break;

            Vector2 currentDirection = new Vector2(0.0, 0.0);
            if (waypoint.parent != null)
            {
                currentDirection = waypoint.position.subtractedBy(waypoint.parent.position).normalized();
            }
            
            for (Waypoint neighbor : waypoint.neighbors)
            {
                if (neighbor.explored || neighbor.blocked) continue;
                //double newPathCost = waypoint.pathCost + Distance(waypoint, neighbor);
                double newPathCost = 0.0;
                double newGoalCost = Distance(neighbor, end);

                Vector2 directionToNeighbor = neighbor.position.subtractedBy(waypoint.position).normalized();
                double newTurnCost = (1.0 - directionToNeighbor.dotProduct(currentDirection)) * turnResistance;

                double newTotalCost = newPathCost + newGoalCost + newTurnCost;
                if (newTotalCost < neighbor.totalCost || !neighbor.onFrontier)
                {
                    neighbor.pathCost = newPathCost;
                    neighbor.goalCost = newGoalCost;
                    neighbor.turnCost = newTurnCost;
                    neighbor.totalCost = newTotalCost;
                    neighbor.parent = waypoint;
                    if (!neighbor.onFrontier)
                    {
                        neighbor.onFrontier = true;
                        frontier.add(neighbor);
                    }
                }
            }
        }
        //Construct final pathway
        ArrayList<Waypoint> path = new ArrayList<Waypoint>();
        if (waypoint == end)
        {
            while (waypoint != null)
            {
                path.add(0, waypoint);
                waypoint = waypoint.parent;
            }
        }
        else
        {
            System.out.println("NO PATH FOUND!");
        }
        //Reset the state
        for (Waypoint wp : frontier)
        {
            wp.Reset();
        }
        for (Waypoint wp : explored)
        {
            wp.Reset();
        }
        return path;
    }
    /**Returns the Euclidean distance from the centers of the two waypoints */
    public double Distance(Waypoint a, Waypoint b)
    {
        return a.position.subtractedBy(b.position).length();
    }

    public synchronized void LoadFromXMLFile(String filePath)
    {
        waypoints.clear();
        targets.clear();

        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
        try
        {
            DocumentBuilder docBuilder = dbf.newDocumentBuilder();
            Document document = docBuilder.parse(filePath);
            Element root = document.getDocumentElement();
            NodeList list = root.getElementsByTagName("waypoint");
            //Load in the waypoints initially
            for (int i = 0; i < list.getLength(); i++)
            {
                Node node = list.item(i);
                NamedNodeMap attributes = node.getAttributes();
                Waypoint waypoint = new Waypoint(
                    Double.parseDouble(attributes.getNamedItem("x").getNodeValue()),
                    Double.parseDouble(attributes.getNamedItem("y").getNodeValue())
                );
                waypoints.add(waypoint);
            }
            //Now assign their neighbors
            for (int i = 0; i < list.getLength(); i++)
            {
                Node wpNode = list.item(i);
                Element wpElem = (Element) wpNode;
                NodeList neighborNodes = wpElem.getElementsByTagName("neighbor");
                for (int j = 0; j < neighborNodes.getLength(); j++)
                {
                    Node nNode = neighborNodes.item(j);
                    NamedNodeMap attributes = nNode.getAttributes();
                    int id = Integer.parseInt(attributes.getNamedItem("waypoint_id").getNodeValue());
                    Waypoint neighbor = waypoints.get(id);
                    waypoints.get(i).neighbors.add(neighbor);
                }
            }
            //Now assign target names
            list = root.getElementsByTagName("target");
            for (int i = 0; i < list.getLength(); i++)
            {
                NamedNodeMap attributes = list.item(i).getAttributes();
                int id = Integer.parseInt(attributes.getNamedItem("waypoint_id").getNodeValue());
                Waypoint waypoint = waypoints.get(id);
                waypoint.targetName = attributes.getNamedItem("name").getNodeValue();
                targets.put(waypoint.targetName, waypoint);
            }
        }
        catch (Exception e)
        {
            System.out.println(e.getMessage());
        }
    }

    public synchronized void SaveAsXMLFile(String filePath)
    {
        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
        try 
        {
            DocumentBuilder docBuilder = dbf.newDocumentBuilder();
            Document document = docBuilder.newDocument();
            Element root = document.createElement("lattice");
            for (Waypoint waypoint : waypoints)
            {
                Element wpElement = document.createElement("waypoint");
                wpElement.setAttribute("x", String.valueOf(waypoint.position.x));
                wpElement.setAttribute("y", String.valueOf(waypoint.position.y));
                
                for (Waypoint neighbor : waypoint.neighbors)
                {
                    Element nElement = document.createElement("neighbor");
                    nElement.setAttribute("waypoint_id", String.valueOf(waypoints.indexOf(neighbor)));
                    wpElement.appendChild(nElement);
                }
                
                root.appendChild(wpElement);
                
                if (!waypoint.targetName.isEmpty())
                {
                    Element targetElement = document.createElement("target");
                    targetElement.setAttribute("name", waypoint.targetName);
                    targetElement.setAttribute("waypoint_id", String.valueOf(waypoints.indexOf(waypoint)));
                    root.appendChild(targetElement);
                }
            }
            document.appendChild(root);

            try
            {
                Transformer trans = TransformerFactory.newInstance().newTransformer();
                trans.setOutputProperty(OutputKeys.INDENT, "yes");
                trans.setOutputProperty(OutputKeys.METHOD, "xml");
                trans.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
                trans.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "4");
                trans.transform(new DOMSource(document), new StreamResult(new FileOutputStream(filePath)));
            }
            catch (Exception e)
            {
                System.out.println(e.getMessage());
            }
        }
        catch (Exception e)
        {
            System.out.println(e.getMessage());
        }
    }
}