package teraLeg;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class TeraLegSimulation {
	private SimulationConstructionSet sim;

	public TeraLegSimulation() {
		TeraLegRobot robot = new TeraLegRobot("rob");
		sim = new SimulationConstructionSet(robot);
	    robot.setController(new TeraLegController(robot));
		sim.setSimulateDuration(2.4);
		Thread teraThread = new Thread(sim);
		teraThread.start();
	}

	public static void main(String[] args) {

		TeraLegSimulation teraSim = new TeraLegSimulation();
	}

}