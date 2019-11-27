//package teraLeg;
//
//public class TeraLegSimulation {
//
//	public static void main(String[] args) {
//		// TODO Auto-generated method stub
//
//	}
//
//}

package teraLeg;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class TeraLegSimulation {
	private SimulationConstructionSet sim;

	public TeraLegSimulation() {
		TeraLegRobot robot = new TeraLegRobot("rob");
		sim = new SimulationConstructionSet(robot);
		Thread teraThread = new Thread(sim);
		teraThread.start();
	}

	public static void main(String[] args) {

		TeraLegSimulation teraSim = new TeraLegSimulation();
	}

}