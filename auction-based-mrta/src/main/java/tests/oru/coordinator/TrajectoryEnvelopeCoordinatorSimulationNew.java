package tests.oru.coordinator;

import java.util.HashSet;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public class TrajectoryEnvelopeCoordinatorSimulationNew extends TrajectoryEnvelopeCoordinatorSimulation{
	
	
	public TrajectoryEnvelopeCoordinatorSimulationNew(double mAX_VEL, double mAX_ACCEL) {
		super(mAX_VEL,mAX_ACCEL);
	}

	public HashSet<CriticalSection> getAllCriticalSections(){
		return super.allCriticalSections;
	}

}
