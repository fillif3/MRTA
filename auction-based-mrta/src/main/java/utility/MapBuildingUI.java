package utility;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.PathEditor2;

public class MapBuildingUI {
	
	public static void makeMapGUI() {
		//Dummy footprint (should be substituted with HX01 footprint)
		Coordinate footprint1 = new Coordinate(-3.6,-3.6);
		Coordinate footprint2 = new Coordinate(3.6,-3.6);
		Coordinate footprint3 = new Coordinate(3.6,3.6);
		Coordinate footprint4 = new Coordinate(-3.6,3.6);
		
		//Volvo CE
		String locAndPathFilename = "output1/locations_and_paths.txt";
		String mapFilename = "maps/elsite_1m.yaml";
		// FPA: Deprecated...
		//PathEditor2 pe = new PathEditor2(locAndPathFilename,mapFilename);
		//pe.setPathPlanningFootprint(footprint1,footprint2,footprint3,footprint4,footprint1);
		PathEditor2 pe = new PathEditor2(locAndPathFilename,mapFilename,footprint1,footprint2,footprint3,footprint4);
		
	}
	
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
		Coordinate footprint1 = new Coordinate(-4,-4);
		Coordinate footprint2 = new Coordinate(4,-4);
		Coordinate footprint3 = new Coordinate(4,4);
		Coordinate footprint4 = new Coordinate(-4,4);
		
		tec.setDefaultFootprint(footprint1,footprint2,footprint3,footprint4);

		String locAndPathFilename = "output1/locations_and_paths.txt";
		Missions.loadLocationAndPathData(locAndPathFilename);
		tec.placeRobot(1, Missions.getLocation("loc_1"));
		
		Missions.enqueueMission(new Mission(1, Missions.getShortestPath("loc_1","loc_2")));
		Missions.enqueueMission(new Mission(1, Missions.getShortestPath("loc_2","loc_3")));
		Missions.enqueueMission(new Mission(1, Missions.getShortestPath("loc_3","loc_4")));
		Missions.enqueueMission(new Mission(1, Missions.getShortestPath("loc_4","loc_5")));
		Missions.enqueueMission(new Mission(1, Missions.getShortestPath("loc_5","loc_7")));
		Missions.enqueueMission(new Mission(1, Missions.getShortestPath("loc_7","loc_8")));
		Missions.enqueueMission(new Mission(1, Missions.getShortestPath("loc_8","loc_2")));		
		Missions.enqueueMission(new Mission(1, Missions.getShortestPath("loc_2","loc_3")));
		Missions.enqueueMission(new Mission(1, Missions.getShortestPath("loc_3","loc_7")));
		Missions.enqueueMission(new Mission(1, Missions.getShortestPath("loc_7","loc_8")));
		Missions.enqueueMission(new Mission(1, Missions.getShortestPath("loc_8","loc_2")));
		
		while (true) {
			Mission currentMission = Missions.peekMission(1);
			if (tec.addMissions(currentMission)) {
				Missions.dequeueMission(1);
				// FPA: you no longer "add missions" and then "start missions". If added,
				// a mission is started, and that's it. If you need to check whether you "could"
				// add a mission before actually adding it, use the tec.isFree(robotID) method.
				//tec.computeCriticalSectionsAndStartTrackingAddedMission();
				Missions.enqueueMission(currentMission);
			}
			System.out.println(tec.getRobotReport(1));
			Thread.sleep(1000);
		}	
	}
	
	public static void main(String[] args) throws InterruptedException {
		
		makeMapGUI();
		//runCoord();
	}
}

