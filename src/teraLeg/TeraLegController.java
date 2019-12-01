package teraLeg;

//import com.yobotics.simulationconstructionset.*;
//import javax.vecmath.*;
import java.util.ArrayList;

import javax.vecmath.Vector3d;

//import teraLeg.TeraLegController.State;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.Axis;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

/**
 * <p>
 * Title: Muar
 * </p>
 *
 * <p>
 * Description: Pata exoesqueleto
 * </p>
 *
 * <p>
 * Copyright: Copyright (c) 2010
 * </p>
 *
 * <p>
 * Company: CAR
 * </p>
 *
 * @author Ant�nio Bento Filho
 * @version 1.0
 */
public class TeraLegController implements RobotController {

	private TeraLegRobot robot;
	private YoVariableRegistry registry;
	private DoubleYoVariable q_x, qd_x, q_z;
	private DoubleYoVariable q_pitch, qd_pitch;
	private DoubleYoVariable q_J1, q_J2, q_J3, q_J4;
	private DoubleYoVariable qd_J1, qd_J2, qd_J3, qd_J4;
	private DoubleYoVariable tau_J1, tau_J2, tau_J3, tau_J4;
	private DoubleYoVariable spring0_z, spring4_z;
	private DoubleYoVariable spring0_dz, spring4_dz;
	private DoubleYoVariable spring0_x, spring4_x;
	private DoubleYoVariable spring0_dx, spring4_dx;
	private DoubleYoVariable spring0_fz, spring4_fz;
	private DoubleYoVariable springForce, z_flight_ant;
	private DoubleYoVariable gc_front, gc_rear;
	private DoubleYoVariable gc_front_fs, gc_rear_fs;
	private DoubleYoVariable gc_front_fz, gc_rear_fz;

	private enum State {
		FLIGHT, GROUND_CONTACT, COMPRESSED
	}

	private State state;

//
	double g = 9.81; // gravity
	double tImp = 0.25; // [s] tiempo para imponer la fuerza de impulso en salto
	double Kleg;
	double v_x = 0.0;
	double Lmax = 1.2; // m�xima longitud de la pierna sin singularidad
	double Lmin = 0.8; // Lmax - h_Des
	double h_Des = 0.4; // altura deseada para control de salto
	double K_LgStf = 10; // cons de moelle de la pierna K_LgStf*m^0.67

	double B_SwLeg = 50; // cons de amort cont posicion del COM respecto al pie
	private TeraLegRobot rob;

	public TeraLegController(TeraLegRobot robot) {
		this.rob = robot;
		registry = new YoVariableRegistry("registry");
		q_x = (DoubleYoVariable) rob.getVariable("q_x");
		qd_x = (DoubleYoVariable) rob.getVariable("qd_x");
		q_z = (DoubleYoVariable) rob.getVariable("q_z");
		q_pitch = (DoubleYoVariable)rob.getVariable("q_pitch");
		qd_pitch = (DoubleYoVariable)rob.getVariable("qd_pitch");
		q_J1 = (DoubleYoVariable)rob.getVariable("q_J1");
		qd_J1 = (DoubleYoVariable)rob.getVariable("qd_J1");
		q_J2 = (DoubleYoVariable)rob.getVariable("q_J2");
		qd_J2 = (DoubleYoVariable)rob.getVariable("qd_J2");
		q_J3 = (DoubleYoVariable)rob.getVariable("q_J3");
		qd_J3 = (DoubleYoVariable)rob.getVariable("qd_J3");
		q_J4 = (DoubleYoVariable)rob.getVariable("q_J4");
		qd_J4 = (DoubleYoVariable)rob.getVariable("qd_J4");
		tau_J1 = (DoubleYoVariable)rob.getVariable("tau_J1");
		tau_J2 = (DoubleYoVariable)rob.getVariable("tau_J2");
		tau_J3 = (DoubleYoVariable)rob.getVariable("tau_J3");
		tau_J4 = (DoubleYoVariable)rob.getVariable("tau_J4");
		spring0_x = (DoubleYoVariable)rob.getVariable("spring0_x");
		spring0_dx = (DoubleYoVariable)rob.getVariable("spring0_dx");
		spring0_z = (DoubleYoVariable)rob.getVariable("spring0_z");
		spring0_dz = (DoubleYoVariable)rob.getVariable("spring0_dz");
		spring4_x = (DoubleYoVariable)rob.getVariable("spring4_x");
		spring4_dx = (DoubleYoVariable)rob.getVariable("spring4_dx");
		spring4_z = (DoubleYoVariable)rob.getVariable("spring4_z");
		spring4_dz = (DoubleYoVariable)rob.getVariable("spring4_dz");
		spring4_fz = (DoubleYoVariable)rob.getVariable("spring4_fz");
		spring0_fz = (DoubleYoVariable)rob.getVariable("spring0_fz");
		spring4_fz = (DoubleYoVariable)rob.getVariable("spring4_fz");
		springForce = (DoubleYoVariable)rob.getVariable("springForce");
		z_flight_ant = (DoubleYoVariable)rob.getVariable("z_flight_ant");
		gc_front = (DoubleYoVariable)rob.getVariable("gc_front");
		gc_rear = (DoubleYoVariable)rob.getVariable("gc_rear");
		gc_front_fs = (DoubleYoVariable)rob.getVariable("gc_front_fs");
		gc_rear_fs = (DoubleYoVariable)rob.getVariable("gc_rear_fs");
		gc_front_fz = (DoubleYoVariable)rob.getVariable("gc_front_fz");
		gc_rear_fz = (DoubleYoVariable)rob.getVariable("gc_rear_fz");
		initControl();
	}

	private void initControl() {

		state = State.FLIGHT;
		q_J2.set(-0.49);
		q_J3.set(1.42);
		q_J4.set(-1.6);
		q_z.set(1.2);
		Kleg = LegStiffness();

	}

	public void doControl() {
		legSpring(Kleg);
		BlockJ1();
		SLIPmodel(h_Des, Lmin, v_x); // vx = 0.0

	}

	//
	// De momento mantenemos el movto de la pata en 2D.
	//
	private void BlockJ1() {
		double K = 100.0, B = 10.0;
		tau_J1.set(-K * (q_J1.getDoubleValue()) - B * qd_J1.getDoubleValue());
	}

	private void legSpring(double Kspring) {
		if (spring0_z.getDoubleValue() < Lmax && spring0_z.getDoubleValue() > spring4_z.getDoubleValue()) {
			spring0_fz.set(Kspring * (Lmax - (spring0_z.getDoubleValue())));
		} else {
			spring0_fz.set(0.0);
		}
		springForce.set(spring0_fz.getDoubleValue());
	}

	private double LegStiffness() {
		return K_LgStf * Math.pow(rob.masaTotal(), 0.67);
	}

	//
	// Jacobiano traspuesta, obtencion de pares articulares en funcion
	// de la fuerza cartesiana en el extremo */
	//
	private void SetJointTorques(double Fx, double Fy, double Fz, double Mx, double My, double Mz) {

		tau_J2.set(DesiredTorqueJ2(Fx, Fy, Fz, Mx, My, Mz));
		tau_J3.set(DesiredTorqueJ3(Fx, Fy, Fz, Mx, My, Mz));
		tau_J4.set(DesiredTorqueJ4(Fx, Fy, Fz, Mx, My, Mz));
	}

	private double DesiredTorqueJ2(double Fx, double Fy, double Fz, double Mx, double My, double Mz) {
		double L3, L4, L5;
		double tau = 0;
		L3 = rob.GetLinkLength(2);
		L4 = rob.GetLinkLength(3);
		L5 = rob.GetLinkLength(4);
		tau = -Fx
				* (L3 * Math.cos(q_J2.getDoubleValue()) + L4 * Math.cos(q_J2.getDoubleValue() + q_J3.getDoubleValue())
						+ L5 * Math.cos(q_J2.getDoubleValue() + q_J3.getDoubleValue() + q_J4.getDoubleValue()))
				- Fz * (-L3 * Math.sin(q_J2.getDoubleValue())
						- L4 * Math.sin(q_J2.getDoubleValue() + q_J3.getDoubleValue())
						- L5 * Math.sin(q_J2.getDoubleValue() + q_J3.getDoubleValue() + q_J4.getDoubleValue()))
				+ My;
		return tau;
	}

	private double DesiredTorqueJ3(double Fx, double Fy, double Fz, double Mx, double My, double Mz) {
		double L4, L5;
		double tau = 0;
		L4 = rob.GetLinkLength(3);
		L5 = rob.GetLinkLength(4);
		tau = -Fx
				* (L4 * Math.cos(q_J2.getDoubleValue() + q_J3.getDoubleValue())
						+ L5 * Math.cos(q_J2.getDoubleValue() + q_J3.getDoubleValue() + q_J4.getDoubleValue()))
				- Fz * (-L4 * Math.sin(q_J2.getDoubleValue() + q_J3.getDoubleValue())
						- L5 * Math.sin(q_J2.getDoubleValue() + q_J3.getDoubleValue() + q_J4.getDoubleValue()))
				+ My;
		return tau;
	}

	private double DesiredTorqueJ4(double Fx, double Fy, double Fz, double Mx, double My, double Mz) {
		double L5;
		double tau = 0;
		L5 = rob.GetLinkLength(4);
		tau = -Fx * (L5 * Math.cos(q_J2.getDoubleValue() + q_J3.getDoubleValue() + q_J4.getDoubleValue()))
				- Fz * (-L5 * Math.sin(q_J2.getDoubleValue() + q_J3.getDoubleValue() + q_J4.getDoubleValue())) + My;
		return tau;
	}

	//
	// estados
	//
//	private double FLIGHT() {
//		return 1.0;
//	}
//
//	private double GROUND_CONTACT() {
//		return 2.0;
//	}
//
//	private double COMPRESSED() {
//		return 3.0;
//	}

	private void SLIPmodel(double h_des, double Lmin, double v_x) {

		if (state == State.GROUND_CONTACT) {
			controlBodyAttitude();
			if (q_z.getDoubleValue() <= Lmin) {
				state = State.COMPRESSED;
				System.out.println("COMPRESSED");
			}
		} else if (state == State.COMPRESSED) {
			controlHoppingHeight(h_Des);
			if (gc_front_fs.getDoubleValue() == 0 && gc_rear_fs.getDoubleValue() == 0) {
				z_flight_ant.set(0.0);
				state = State.FLIGHT;
				System.out.println("FLIGHT");
			}
		} else if (state == State.FLIGHT) {
			swingLeg();
			GetMaxFlightHeight();
			if (gc_front_fz.getDoubleValue() >= 5 || gc_rear_fz.getDoubleValue() >= 5) {
				state = State.GROUND_CONTACT;
				System.out.println("GROUND_CONTACT");
			}
		}

	}

	private void swingLeg() {
		tau_J2.set(-B_SwLeg * qd_J2.getDoubleValue());
		tau_J3.set(-B_SwLeg * qd_J3.getDoubleValue());
		tau_J4.set(-B_SwLeg * qd_J4.getDoubleValue());
	}

	private void controlBodyAttitude() {
		double Fx = 0, Fy = 0, Fz = 0, Mx = 0, My = 0, Mz = 0;
		double Kx = 300, Bx = 150, Ky = 50, By = 30;
		//
		// Esto mantiene el pie en la vertical del CM // * Math.sin( -q_J4.val)
		Fx = Kx * (q_x.getDoubleValue() - spring4_x.getDoubleValue()) + Bx * (-spring4_dx.getDoubleValue());
		//
		// Esto corrige el pitch
		My = Ky * q_pitch.getDoubleValue() + By * qd_pitch.getDoubleValue();
		//
		// aplica la transpuesta del jacobiano
		SetJointTorques(Fx, Fy, Fz, Mx, My, Mz);
	}

	private void controlHoppingHeight(double h_des) {

		// double v_i = 0, x_i, K, Fx = 0, Fz, My = 0, M, Kx = 460, Bx = 150;
		double v_i = 0, x_i, K, Fx = 0, Fz, My = 0, M, Kx = 1500, Bx = 150;
		double h_error = 0, delta_h = 0, Ke = 1.14;
		//
		// control de la altura se salto; v_i es la velocidad de impulso,;
		// x_i es la compresi�n inicial de la pierna
		//
		// diferencia entre la altura de salto deseada y la del �ltimo vuelo
		delta_h = h_des - q_z.getDoubleValue() + Ke * h_error;
		delta_h = h_des + Ke * h_error;
		x_i = Lmax - q_z.getDoubleValue(); // la posici�n inicial del muelle.
		K = Kleg;
		M = rob.masaTotal(); // Masa total
		// C�lculo de velocidad inicial vertical para el salto:
		if (K * Math.pow(x_i, 2.0) / M >= 2 * 9.8 * delta_h) {
			v_i = 0.0;
			System.out.println("La energ�a del muelle es suficiente para alcanzar h");
		} else
			v_i = Math.sqrt(2 * 9.8 * delta_h - K * Math.pow(x_i, 2.0) / M);
		// Nos evitamos invertir el jacobiano si lo pasamos a fuerza:
		// El impulso M V = F DT
		// Controlamos tambi�n la posici�n del pie y la orientaci�n del cuerpo
		Fx = Kx * (q_x.getDoubleValue() - spring4_x.getDoubleValue())
				+ Bx * (qd_x.getDoubleValue() - spring4_dx.getDoubleValue());
		// My = 50 * q_pitch.val + 20 * qd_pitch.val;
		My = 1000 * q_pitch.getDoubleValue() + 400 * qd_pitch.getDoubleValue();
		// dt=0.02 es el periodo de muestreo; repartimos el impulso en 10 dT, es decir,
		// 0.2s
		Fz = -0.1 * M * v_i / 0.02;
		// torques nas articulaciones
		SetJointTorques(Fx, 0, Fz, 0, My, 0); // Aplica la traspuesta del jacobiano
	}

	private void GetMaxFlightHeight() {
		z_flight_ant.set(Math.max(q_z.getDoubleValue(), z_flight_ant.getDoubleValue()));
		return;
	}

	@Override
	public String getDescription() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public YoVariableRegistry getYoVariableRegistry() {
		// TODO Auto-generated method stub
		return registry;
	}

	@Override
	public void initialize() {
		// TODO Auto-generated method stub

	}

}
