package deliaApplication;


import java.util.Date;
import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ConditionObserver;
import com.kuka.roboticsAPI.conditionModel.IConditionListener;
import com.kuka.roboticsAPI.conditionModel.IRisingEdgeListener;
import com.kuka.roboticsAPI.conditionModel.NotificationType;

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
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class Listener extends RoboticsAPIApplication {
	
	final static double corner=Math.toRadians(90);
	
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	private static final int stiffnessZ = 2500; // isolate freedom along z-axis only
	private static final int stiffnessY = 2500;
	private static final int stiffnessX = 2500;
	
	private static double[] homePosition=new double[]{-corner,0,0,-corner,0,corner,0};

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
		DataRecorder blackBox = new DataRecorder("bubblePop",45,TimeUnit.SECONDS,100);
		blackBox.addCartesianForce(lbr.getFlange(),null); // records end-effector force in x,y,z
		blackBox.enable();
		
		/*
		 * Begin recording data and return to starting (home) position
		 */
		getLogger().info("Move to home position");
		PTP returnHome = ptp(homePosition);
		returnHome.setJointVelocityRel(0.3);
		lbr.move(returnHome);
		
		getLogger().info("Start recording");
		blackBox.startRecording();
		
		Frame startFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
		Spline pushDown = Trajectory(startFrame).setJointJerkRel(0.5).setCartVelocity(250);
		
		
		ForceCondition pushAchieved = ForceCondition.createSpatialForceCondition(lbr.getFlange(),10);
		
		IRisingEdgeListener pushListener = new IRisingEdgeListener() {

			@Override
			public void onRisingEdge(ConditionObserver conditionObserver,
					Date time, int missedEvents) {
				getLogger().info("Rivet successfully installed!");
				PTP fetchNext = ptp(homePosition);
				fetchNext.setJointVelocityRel(0.25);
				lbr.move(fetchNext);
				
			}
		};
		
		ConditionObserver pushObserver = getObserverManager().createConditionObserver(pushAchieved, 
				NotificationType.MissedEvents, pushListener);
		pushObserver.enable();
		
		
		/*
		 * Execute straight line motion using the defined impedance controller
		 */
		getLogger().info("Execute straight line motion");
		pushDown.setJointVelocityRel(0.4);
		lbr.move(pushDown.setMode(impedanceControlMode));
		
		getLogger().info("Stop recording");
		blackBox.stopRecording();
	}
		
	private Spline Trajectory(Frame centreFrame){
		Frame down = (new Frame(centreFrame)).setX(-120).setY(20).setZ(560);
			
		Spline zPush = new Spline(
				spl(down));
		return zPush;
	}
	
	private Spline Fetch(Frame centreFrame){
		Frame up = (new Frame(centreFrame)).setX(-120).setY(20).setZ(400);
			
		Spline fetch = new Spline(
				spl(up));
		return fetch;
	}



	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		Listener app = new Listener();
		app.runApplication();
	}
}
