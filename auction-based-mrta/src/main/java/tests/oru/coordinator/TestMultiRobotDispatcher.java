package tests.oru.coordinator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.Vector;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.PathEditor2;
import utility.TopologicalMap;

public class TestMultiRobotDispatcher {

	public static void runCoord() throws InterruptedException {
		double MAX_ACCEL = 10.0;
		double MAX_VEL = 34.0;
		int CONTROL_PERIOD = 1000;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD,1000.0,MAX_VEL,MAX_ACCEL);
		//		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);

		tec.setupSolver(0, 100000000);
		String mapFilename = "maps/elsite_1m.yaml";
		BrowserVisualization viz = new BrowserVisualization();
		viz.setMap(mapFilename);
		viz.setInitialTransform(3.5, 55.0, 6.5);
		tec.setVisualization(viz);

		//Dummy footprint (should be substituted with HX01 footprint)
		Coordinate footprint1 = new Coordinate(-2,-2);
		Coordinate footprint2 = new Coordinate(2,-2);
		Coordinate footprint3 = new Coordinate(2,2);
		Coordinate footprint4 = new Coordinate(-2,2);

		tec.setDefaultFootprint(footprint1,footprint2,footprint3,footprint4);

		String locAndPathFilename = "output1/locations_and_paths.txt";
		Missions.loadLocationAndPathData(locAndPathFilename);
		
		HashMap<Integer, Mission> currentMissions = new HashMap<Integer, Mission>(); 
		
		//can be up to 8
		int robotNumber = 6;
		//parking
		Vector<Mission> parkingMissions = new Vector<Mission>(); 
		for (int i = 1; i <= robotNumber; i++) {
			tec.placeRobot(i, Missions.getLocation("P_"+i));
			Mission m = new Mission(i, "P_"+i,"loc_2", Missions.getShortestPath("P_"+i,"loc_2"));
			Missions.enqueueMission(m);
			parkingMissions.add(m);
			currentMissions.put(i, m);
		}

		HashMap<TopologicalMap, Mission> topToMis = new HashMap<TopologicalMap, Mission>(); 
		for (int i = 1; i <= robotNumber; i++) {
			createTopoligicalMissions(topToMis, i, "loc_2","loc_3");
			createTopoligicalMissions(topToMis, i, "loc_3","loc_4");
			createTopoligicalMissions(topToMis, i, "loc_4","loc_5");
			createTopoligicalMissions(topToMis, i, "loc_5","loc_7");
			createTopoligicalMissions(topToMis, i, "loc_7","loc_8");
			createTopoligicalMissions(topToMis, i, "loc_8","loc_2");
			createTopoligicalMissions(topToMis, i, "loc_3","loc_7");	
		}


		//dispatching from Parking
		for (Mission mission : parkingMissions) {
			if (tec.addMissions(mission)) {
				
				//tec.computeCriticalSectionsAndStartTrackingAddedMission();
				Missions.dequeueMission(mission.getRobotID());
			}
		}
		
		while (true) {
			for (int i = 1; i <= robotNumber; i++) {
				Mission currentMission = currentMissions.get(i);
				Mission nextMission = getNextMission(currentMission, topToMis);
				if (tec.addMissions(nextMission)) {
					Missions.dequeueMission(i);
					// FPA: you no longer "add missions" and then "start missions". If added,
					// a mission is started, and that's it. If you need to check whether you "could"
					// add a mission before actually adding it, use the tec.isFree(robotID) method.
					//tec.computeCriticalSectionsAndStartTrackingAddedMission();					
					Missions.enqueueMission(nextMission);
					currentMissions.put(i, nextMission);
				}
				System.out.println(tec.getRobotReport(1));	
			}
			Thread.sleep(1000);
		}	
	
	}

	private static Mission getNextMission(Mission currentMission, HashMap<TopologicalMap, Mission> topToMis) {
		String locationTo = currentMission.getToLocation();
		Vector<Mission> toBeDispatched = new Vector<Mission>();		
		for (TopologicalMap tm : topToMis.keySet() ) {
			Mission tmp = topToMis.get(tm);			
			if(currentMission.getRobotID() == tmp.getRobotID() &&
					tm.getFrom().compareTo(locationTo) == 0) {
				toBeDispatched.add(tmp);
			}
		}
		if(toBeDispatched.size() == 1) {
			return toBeDispatched.get(0);
		}else {
			System.out.println("*****************************************************************");
			int random = new Random().nextInt(toBeDispatched.size() - 1);
			return toBeDispatched.get(random);			
		}
	}

	private static void createTopoligicalMissions(HashMap<TopologicalMap, Mission> topToMis, int robotId, String from, String to) {
		Mission m = new Mission(robotId, from, to, Missions.getShortestPath(from, to));
		Missions.enqueueMission(m);
		topToMis.put(new TopologicalMap(from, to), m);		
	}

	public static void main(String[] args) throws InterruptedException {

		runCoord();
	}
}

