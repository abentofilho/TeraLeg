package teraLeg;

import java.util.ArrayList;

import javax.vecmath.Vector3d;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.Axis;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class TeraLegRobot extends Robot {
	private final ArrayList<GroundContactPoint> groundContactPoints = new ArrayList();

	private static final double BODY_MASS = 30.0;
	private static final double BODY_LENGTH = 0.150;
	private static final double BODY_RADIUS = 0.050;
	private static final double BODY_Izz = BODY_MASS * Math.pow(BODY_RADIUS, 2) / 2.0;
	private static final double BODY_Iyy = BODY_MASS
			* (Math.pow(BODY_RADIUS, 2) / 2.0 * Math.pow(BODY_LENGTH, 2) / 12.0);
	private static final double BODY_Ixx = BODY_Iyy;

	private static final double HIP_MASS = 1.5;
	private static final double HIP_LENGTH = 0.150;
	private static final double HIP_RADIUS = 0.040;
	private static final double HIP_Izz = HIP_MASS * Math.pow(HIP_RADIUS, 2) / 2.0;
	private static final double HIP_Iyy = HIP_MASS * (Math.pow(HIP_RADIUS, 2) / 2.0 * Math.pow(HIP_LENGTH, 2) / 12.0);
	private static final double HIP_Ixx = HIP_Iyy;
	private static final double HIP_COM_X = 0.00, HIP_COM_Y = 0.00, HIP_COM_Z = -HIP_LENGTH / 2;

	private static final double THIGH_MASS = 1.5;
	private static final double THIGH_LENGTH = 0.400;
	private static final double THIGH_RADIUS = 0.030;
	private static final double THIGH_Izz = THIGH_MASS * Math.pow(THIGH_RADIUS, 2) / 2.0;
	private static final double THIGH_Iyy = THIGH_MASS
			* (Math.pow(THIGH_RADIUS, 2) / 2.0 * Math.pow(THIGH_LENGTH, 2) / 12.0);
	private static final double THIGH_Ixx = THIGH_Iyy;
	private static final double THIGH_COM_X = 0.00, THIGH_COM_Y = 0.00, THIGH_COM_Z = -THIGH_LENGTH / 2;

	private static final double SHANK_MASS = 1.0;
	private static final double SHANK_LENGTH = 0.360;
	private static final double SHANK_RADIUS = 0.025;
	private static final double SHANK_Izz = SHANK_MASS * Math.pow(SHANK_RADIUS, 2) / 2.0;
	private static final double SHANK_Iyy = SHANK_MASS
			* (Math.pow(SHANK_RADIUS, 2) / 2.0 * Math.pow(SHANK_LENGTH, 2) / 12.0);
	private static final double SHANK_Ixx = SHANK_Iyy;
	private static final double SHANK_COM_X = 0.00, SHANK_COM_Y = 0.00, SHANK_COM_Z = -SHANK_LENGTH / 2;

	private static final double FOOT_MASS = 0.5;
	private static final double FOOT_LENGTH = 0.190;
	private static final double FOOT_RADIUS = 0.020;
	private static final double FOOT_Izz = FOOT_MASS * Math.pow(FOOT_RADIUS, 2) / 2.0;
	private static final double FOOT_Iyy = FOOT_MASS
			* (Math.pow(FOOT_RADIUS, 2) / 2.0 * Math.pow(FOOT_LENGTH, 2) / 12.0);
	private static final double FOOT_Ixx = FOOT_Iyy;
	private static final double FOOT_COM_X = 0.00, FOOT_COM_Y = 0.00, FOOT_COM_Z = -2 * FOOT_LENGTH / 2;
	static double TOE_RADIUS = 0.006, TOE_HEIGHT = 0.003;

	static double ANKLE_ANGLE = -0.9; // radians
	// joint esthetic
	private static final double JOINT_LENGTH = 0.10;
	private static final double JOINT_RADIUS = 0.03;

	private static final double ROBOT_HEIGHT = HIP_LENGTH + THIGH_LENGTH + SHANK_LENGTH + FOOT_LENGTH;

	private static final int FOOT_LENGHT = 0;

	public TeraLegRobot(String robot) {
		super("robot");
//		this.setGravity(0.0, 0.0, -9.81);
		// junta 0 base
		Joint J0 = new FloatingPlanarJoint("J0", this);
		Link link0 = link0();
		link0.addCoordinateSystemToCOM(FOOT_LENGTH);
		J0.setLink(link0);
		this.addRootJoint(J0);
		// junta 1 abdução do quadril
		PinJoint J1 = new PinJoint("J1", new Vector3d(0.0, 0.0, ROBOT_HEIGHT), this, Axis.X);
		Link link1 = link1();
		link1.addCoordinateSystemToCOM(FOOT_LENGTH);
		J1.setLink(link1);
		J0.addJoint(J1);
		// junta 2 flexão do quadril
		PinJoint J2 = new PinJoint("J2", new Vector3d(0.0, 0.0, -HIP_LENGTH), this, Axis.Y);
		Link link2 = link2();
		J2.setLink(link2);
		J1.addJoint(J2);
		// junta 3 flexão do joelho
		PinJoint J3 = new PinJoint("J3", new Vector3d(0.0, 0.0, -THIGH_LENGTH), this, Axis.Y);
		Link link3 = link3();
		J3.setLink(link3);
		J2.addJoint(J3);
		// junta 4 flexão do tornozelo
		PinJoint J4 = new PinJoint("J4", new Vector3d(0.0, 0.0, -SHANK_LENGTH), this, Axis.Y);
		Link link4 = link4();
		J4.setLink(link4);
		J3.addJoint(J4);
		// pontos de contato com o solo
		// peito do pé
		GroundContactPoint gc_foot = new GroundContactPoint("gc_front",
				new Vector3d(TOE_RADIUS * Math.cos(Math.PI / 2 + ANKLE_ANGLE) - TOE_HEIGHT * Math.cos(-ANKLE_ANGLE),
						0.0, -FOOT_LENGTH + TOE_HEIGHT - TOE_RADIUS * Math.sin(Math.PI / 2 + ANKLE_ANGLE)
								- TOE_HEIGHT * Math.sin(-ANKLE_ANGLE)),
				this);
		groundContactPoints.add(gc_foot);
		J4.addGroundContactPoint(gc_foot);
		// calcanhar
		gc_foot = new GroundContactPoint("gc_rear",
				new Vector3d(-TOE_RADIUS * Math.cos(Math.PI / 2.0 + ANKLE_ANGLE) - TOE_HEIGHT * Math.cos(-ANKLE_ANGLE),
						0.0, -FOOT_LENGTH + TOE_HEIGHT + TOE_RADIUS * Math.sin(Math.PI + ANKLE_ANGLE)
								- TOE_HEIGHT * Math.sin(-ANKLE_ANGLE)),
				this);
		groundContactPoints.add(gc_foot);
		J4.addGroundContactPoint(gc_foot);
		/*
		 * modelo físico dos pontos de contato
		 */
		LinearGroundContactModel ground = new LinearGroundContactModel(this, this.getRobotsYoVariableRegistry());
		/*
		 * propriedades dos contatos
		 */
		ground.setZStiffness(2000.0);
		ground.setZDamping(1500.0);
		ground.setXYStiffness(50000.0);
		ground.setXYDamping(2000.0);
		/*
		 * modelo de perfil de solo
		 */
		ground.setGroundProfile3D(new FlatGroundProfile());
		this.setGroundContactModel(ground);
		/*
		 * Simulamos uma mola entre a ponta do pé e a rootjoint - extremo superior
		 */
		Vector3d spring_pos = new Vector3d(0.0, 0.0, 0.0);
		ExternalForcePoint spring_pt = new ExternalForcePoint("spring0", spring_pos, this);
		J0.addExternalForcePoint(spring_pt);
		/*
		 * Simulamos uma mola entre a ponta do pé e a rootjoint- extremo inferior
		 */
		Vector3d spring_pos2 = new Vector3d(0, 0, -FOOT_LENGHT);
		ExternalForcePoint spring_pt2 = new ExternalForcePoint("spring4", spring_pos2, this);
		J4.addExternalForcePoint(spring_pt2);
	}

	private Link link0() { // base
		Link ret = new Link("link0");
		ret.setMass(BODY_MASS);
		ret.setComOffset(0.0, 0.0, 0.0);
		ret.setMomentOfInertia(BODY_Ixx, BODY_Iyy, BODY_Izz);
		return ret;
	}

	private Link link1() { // hip
		Link ret = new Link("link1");
		ret.setMass(HIP_MASS);
		ret.setMomentOfInertia(HIP_Ixx, HIP_Iyy, HIP_Izz);
		ret.setComOffset(HIP_COM_X, HIP_COM_Y, HIP_COM_Z);
		Graphics3DObject linkGraphics = new Graphics3DObject();
		linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
		linkGraphics.translate(0.0, 0.0, -JOINT_LENGTH / 2.0);
		linkGraphics.addCylinder(JOINT_LENGTH, JOINT_RADIUS, YoAppearance.AluminumMaterial());
		linkGraphics.identity();
		linkGraphics.translate(0.0, 0.0, -HIP_LENGTH);
		linkGraphics.addCylinder(HIP_LENGTH, HIP_RADIUS, YoAppearance.Red());
		ret.setLinkGraphics(linkGraphics);
		return ret;
	}

	private Link link2() { // thigh
		Link ret = new Link("link2");
		ret.setMass(THIGH_MASS);
		ret.setMomentOfInertia(THIGH_Ixx, THIGH_Iyy, THIGH_Izz);
		ret.setComOffset(THIGH_COM_X, THIGH_COM_Y, THIGH_COM_Z);
		Graphics3DObject linkGraphics = new Graphics3DObject();
		linkGraphics.rotate(Math.PI / 2.0, Axis.X);
		linkGraphics.translate(0.0, 0.0, -JOINT_LENGTH / 2.0);
		linkGraphics.addCylinder(JOINT_LENGTH, JOINT_RADIUS, YoAppearance.AluminumMaterial());
		linkGraphics.identity();
		linkGraphics.translate(0.0, 0.0, -THIGH_LENGTH);
		linkGraphics.addCylinder(THIGH_LENGTH, THIGH_RADIUS, YoAppearance.Blue());
		ret.setLinkGraphics(linkGraphics);
		return ret;
	}

	private Link link3() { // SHANK
		Link ret = new Link("link3");
		ret.setMass(SHANK_MASS);
		ret.setMomentOfInertia(SHANK_Ixx, SHANK_Iyy, SHANK_Izz);
		ret.setComOffset(SHANK_COM_X, SHANK_COM_Y, SHANK_COM_Z);
		Graphics3DObject linkGraphics = new Graphics3DObject();
		linkGraphics.rotate(Math.PI / 2.0, Axis.X);
		linkGraphics.translate(0.0, 0.0, -JOINT_LENGTH / 2.0);
		linkGraphics.addCylinder(JOINT_LENGTH, JOINT_RADIUS, YoAppearance.AluminumMaterial());
		linkGraphics.identity();
		linkGraphics.translate(0.0, 0.0, -SHANK_LENGTH);
		linkGraphics.addCylinder(SHANK_LENGTH, SHANK_RADIUS, YoAppearance.Green());
		ret.setLinkGraphics(linkGraphics);
		return ret;
	}

	private Link link4() { // foot
		Link ret = new Link("link4");
		ret.setMass(FOOT_MASS);
		ret.setMomentOfInertia(FOOT_Ixx, FOOT_Iyy, FOOT_Izz);
		ret.setComOffset(FOOT_COM_X, FOOT_COM_Y, FOOT_COM_Z);
		Graphics3DObject linkGraphics = new Graphics3DObject();
		linkGraphics.rotate(Math.PI / 2.0, Axis.X);
		linkGraphics.translate(0.0, 0.0, -JOINT_LENGTH / 2.0);
		linkGraphics.addCylinder(JOINT_LENGTH, JOINT_RADIUS, YoAppearance.AluminumMaterial());
		linkGraphics.identity();
		linkGraphics.translate(0.0, 0.0, -FOOT_LENGTH);
		linkGraphics.addCylinder(FOOT_LENGTH, FOOT_RADIUS, YoAppearance.Yellow());
		linkGraphics.rotate(Math.PI / 2 + ANKLE_ANGLE, Axis.Y);
		linkGraphics.addCylinder(FOOT_LENGTH / 10, FOOT_RADIUS * 4, YoAppearance.Black());
		ret.setLinkGraphics(linkGraphics);
		return ret;
	}

	public double GetLinkLength(int link) {
		double select = 0;
		switch (link) {
		case 0:
			select= BODY_LENGTH;
		case 1:
			select= HIP_LENGTH;
		case 2:
			select= THIGH_LENGTH;
		case 3:
			select= SHANK_LENGTH;
		case 4:
			select= FOOT_LENGTH;
		}
		return select;
	}

	public double masaTotal() {
		double massa;
		return massa = BODY_MASS + HIP_MASS + THIGH_MASS + SHANK_MASS + FOOT_MASS;
	}

}
