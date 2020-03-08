package tests.oru.coordinator;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Comparator;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.PathEditor2;

public class TestExternalMotionPlanner1 {

	public static void runCoord() throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		
		// FPA: trajectory envelope trackers are no longer visible from the outside, as it
		// FPA: was dangerous to give access to them. They are not really needed anyway for
		// FPA: purposes outside the scope of the coordinator.
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				//RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
				//RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
				RobotReport robotReport1 = o1.getRobotReport();
				RobotReport robotReport2 = o2.getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				//return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()-o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
				return (o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
			}
		});
				
		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		// FPA: minor API change to make the orders of parameters consistent everywhere...
		//tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
		//tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));

		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = "maps/map.yaml";
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile);
		tec.setVisualization(viz);
		



		tec.setUseInternalCriticalPoints(false);
		tec.setBreakDeadlocks(true);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.05);
		rsp.setRadius(0.5);
		rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);

		int width = 640;    //width of the image 
        int height = 400;   //height of the image 
  
        // For storing image in RAM 
        BufferedImage image2 = null; 
		
		try
        { 
            File input_file = new File("maps/map-partial-dil.png"); //image file path 
            /* create an object of BufferedImage type and pass 
               as parameter the width,  height and image int 
               type.TYPE_INT_ARGB means that we are representing 
               the Alpha, Red, Green and Blue component of the 
               image pixel using 8 bit integer value. */
            image2 = new BufferedImage(width, height, 
                                   BufferedImage.TYPE_BYTE_BINARY); 
  
             // Reading input file 
            image2 = ImageIO.read(input_file); 
            
           // image2 = ImageIO.read(input_file);
  
            System.out.println("Reading complete."); 
        } 
		catch(IOException e) 
        { 
            System.out.println("Error: "+e); 
        } 
		
		Pose[] startPoses = new Pose[2];
		Pose[] goalPoses = new Pose[2];

		Pose startPoseRobot1 = new Pose(32.0,28.0,0.0);
		Pose goalPoseRobot1 = new Pose(5.0,1.0,0.0);
		Pose startPoseRobot2 = new Pose(2.0,38.0,0.0);
		Pose goalPoseRobot2 = new Pose(2.5,3.0,0.0);
		
		Pose[] sposes = image.createFreePoses(width,height,image2,4,0.1);
		

		
		startPoses[0] = sposes[0];
		startPoses[1] = sposes[1];;
		goalPoses[0] = sposes[2];;
		goalPoses[1] = sposes[3];;
		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		
		// Creating task allocation
		
		//int[] alloc = Allocation.allocation(startPoses,goalPoses,rsp,footprint1, footprint2, footprint3, footprint4);
		 int[] alloc = {0,1};
		
			tec.placeRobot(1, startPoses[0]);
			tec.placeRobot(2, startPoses[1]);
		

		rsp.setStart(startPoses[0]);
		rsp.setGoals(goalPoses[alloc[0]]);
		Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
		rsp.addObstacles(fpGeom, startPoses[1], goalPoses[alloc[1]]);
		if (!rsp.plan()) throw new Error ("No path between " + startPoses[0] + " and " + goalPoses[alloc[0]]);
		PoseSteering[] pss1 = rsp.getPath();

		
		

		rsp.setStart(startPoses[1]);
		rsp.setGoals(goalPoses[alloc[1]]);
		rsp.clearObstacles();
		//rsp.addObstacles(fpGeom, startPoses[0], goalPoses[alloc[0]]);
		if (!rsp.plan()) throw new Error ("No path between " + startPoses[1] + " and " + goalPoses[alloc[1]]);
		PoseSteering[] pss2 = rsp.getPath();

		Missions.enqueueMission(new Mission(1, pss1));
		//Missions.enqueueMission(new Mission(1, pss1Inv));

		Missions.enqueueMission(new Mission(2, pss2));
		//Missions.enqueueMission(new Mission(2, pss2Inv));

		System.out.println("Added missions " + Missions.getMissions());
		
		
		
		boolean[] freeRobots = new boolean[2];

		
		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 1; i <= 2; i++) {
			final int robotID = i;
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				@Override
				public void run() {
					while (true) {
						//Mission to dispatch alternates between (rip -> desti) and (desti -> rip)
						Mission m = Missions.getMission(robotID, iteration%2);
						synchronized(tec) {
							//addMission returns true iff the robot was free to accept a new mission
							if (tec.addMissions(m)) {
								
								
								
								// FPA: you no longer "add missions" and then "start missions". If added,
								// a mission is started, and that's it. If you need to check whether you "could"
								// add a mission before actually adding it, use the tec.isFree(robotID) method.
								//tec.computeCriticalSections();
								//tec.startTrackingAddedMissions();
								iteration++;
							}
						}
						//Sleep for a little (2 sec)
						try { Thread.sleep(2000); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
				}
			};
			//Start the thread!
			t.start();
		}
		
		
		while(true)
		{
			for(int i=0;i<2;i++) freeRobots[i] = tec.isFree(i+1);
				
			
			System.out.println("start");
			if (Allocation.contains(freeRobots, false)>0)
			{
				
				System.out.println("koniec ");
				break;
			}
		}
		
		while(true)
		{
			for(int i=0;i<2;i++) freeRobots[i] = tec.isFree(i+1);
				
			
			System.out.println("start");
			if (Allocation.contains(freeRobots, false)==0)
			{
				
				System.out.println("koniec ");
				break;
			}
		}
		
		tec.placeRobot(1, startPoses[0]);
		tec.placeRobot(2, startPoses[1]);
		tec.
		

	}

	public static void main(String[] args) throws InterruptedException {

		runCoord();
	}
}

