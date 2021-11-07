package deliaApplication;


import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class Compliance extends RoboticsAPIApplication {
	
	final static double corner=Math.toRadians(90);
	
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	private static final int stiffnessZ = 300; // isolate freedom along z-axis only
	private static final int stiffnessY = 2500;
	private static final int stiffnessX = 2500;
	
	private static double[] homePosition=new double[]{0,0,0,-corner,0,corner,0};

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
	}

	public void run() {
		/*
		 * Defining the impedance controlled based on initialised stiffness values
		 */
		CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		
		/*
		 * Initialising the data recorder and setting what to record
		 */
		DataRecorder blackBox = new DataRecorder("28MayTest",45,TimeUnit.SECONDS,100);
//		blackBox.addInternalJointTorque(lbr);
//		blackBox.addExternalJointTorque(lbr);
		blackBox.addCartesianForce(lbr.getFlange(),null); // records end-effector force in x,y,z
//		blackBox.addCartesianTorque(lbr.getFlange(),null);
		blackBox.addCommandedCartesianPositionXYZ(lbr.getFlange(), lbr.getRootFrame());
		blackBox.addCurrentCartesianPositionXYZ(lbr.getFlange(), lbr.getRootFrame());
		blackBox.enable();
		
		/*
		 * Begin recording data and return to starting (home) position
		 */
		getLogger().info("Move to home position");
		PTP returnHome = ptp(homePosition);
		returnHome.setJointVelocityRel(0.3);
		lbr.move(returnHome);
		getLogger().info("At home");
		blackBox.startRecording();
		
		/*
		 * Calculate spline motion
		 */
		getLogger().info("Compute spline for straight line motion");
		Frame startFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
		Spline straightLine = straightLineTrajectory(startFrame).setJointJerkRel(0.5).setCartVelocity(250);
		
		/*
		 * Execute straight line motion using the defined impedance controller
		 */
		getLogger().info("Execute straight line motion");
		straightLine.setJointVelocityRel(0.15);
		lbr.move(straightLine.setMode(impedanceControlMode));
		blackBox.stopRecording();
		
		/*
		 * Print the location of the data recorder file
		 */
		System.out.println(blackBox.getURL());
	}

	
	private Spline straightLineTrajectory(Frame centreFrame) {
		Frame downwardFrame = (new Frame(centreFrame)).setX(-120).setY(0).setZ(275);
		Frame forwardFrame =(new Frame(centreFrame)).setX(-300).setY(0).setZ(275);
		
		Spline straightLine = new Spline(
				spl(downwardFrame),
				spl(downwardFrame),
				spl(forwardFrame));
		return straightLine;
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		straightLineTraj app = new straightLineTraj();
		app.runApplication();
	}
}