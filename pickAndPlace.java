package deliaApplication;

import java.util.Date;
import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
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
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class pickAndPlace extends RoboticsAPIApplication {
	
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	private static final int stiffnessZ = 2500; // isolate freedom along z-axis only
	private static final int stiffnessY = 2500;
	private static final int stiffnessX = 2500;
	
	private static double[] homePosition=new double[]{Math.toRadians(-90),Math.toRadians(38.47),0,Math.toRadians(-109.08),
		0,Math.toRadians(-53.02),Math.toRadians(142.93)};
	
	private static double[] pickPosition1=new double[]{Math.toRadians(-46.43),Math.toRadians(68.68),Math.toRadians(0.25),
		Math.toRadians(-99.65),Math.toRadians(-4.65),Math.toRadians(-72.64),Math.toRadians(129.77)};
	
	private static double[] pickPosition2=new double[]{Math.toRadians(-46.43),Math.toRadians(70.51),Math.toRadians(0.25),
		Math.toRadians(-98.29),Math.toRadians(-4.92),Math.toRadians(-72.70),Math.toRadians(129.77)};
	
	private static double[] lift=new double[]{Math.toRadians(-46.43),Math.toRadians(70.76),Math.toRadians(0),
		Math.toRadians(-99.15),Math.toRadians(-4.64),Math.toRadians(-77.90),Math.toRadians(129.77)};
	
	private static double[] placeApproach=new double[]{Math.toRadians(-56.07),Math.toRadians(69.07), Math.toRadians(1.95),
		Math.toRadians(-119.99),Math.toRadians(-16.19),Math.toRadians(-106.64),Math.toRadians(117.21)};
	
	private static double[] placeApproachCloser=new double[]{Math.toRadians(-57.46),Math.toRadians(69.07), Math.toRadians(4.75),
		Math.toRadians(-118.31),Math.toRadians(-14.74),Math.toRadians(-93.25),Math.toRadians(115.96)};

	private static double[] pushIn=new double[]{Math.toRadians(-56.07),Math.toRadians(69.07), Math.toRadians(1.95),
		Math.toRadians(-117.76),Math.toRadians(-17.54),Math.toRadians(-85),Math.toRadians(117.21)};
	
	private static double[] pushInNew=new double[]{Math.toRadians(-57.5),Math.toRadians(74.84), Math.toRadians(3.39),
		Math.toRadians(-115.65),Math.toRadians(-18.53),Math.toRadians(-99.41),Math.toRadians(115.85)};
	
	private static double[] revisedPick=new double[]{Math.toRadians(-162.65),Math.toRadians(81.97), Math.toRadians(36.51),
		Math.toRadians(-113.47),Math.toRadians(-66.78),Math.toRadians(-55.44),Math.toRadians(101)};
	
	private static double[] suction=new double[]{Math.toRadians(-162.65),Math.toRadians(82.77), Math.toRadians(37.34),
		Math.toRadians(-113.20),Math.toRadians(-66.22),Math.toRadians(-53.01),Math.toRadians(100.92)};
	
	private static double[] waypoint=new double[]{Math.toRadians(-142.11),Math.toRadians(53.70), Math.toRadians(30.87),
		Math.toRadians(-115.07),Math.toRadians(-55.18),Math.toRadians(-62.08),Math.toRadians(104.74)};

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
		
		DataRecorder blackBox = new DataRecorder("pickAndInstall",45,TimeUnit.SECONDS,100);
		blackBox.addCartesianForce(lbr.getFlange(),null); // records end-effector force in x,y,z
		blackBox.enable();
		

		getLogger().info("Starting at home.");
		PTP returnHome = ptp(homePosition);
		returnHome.setJointVelocityRel(0.1);
		lbr.move(returnHome);
		
		getLogger().info("Start recording");
		blackBox.startRecording();
		
		getLogger().info("On the way...");
		PTP stopover = ptp(waypoint);
		stopover.setJointVelocityRel(0.1);
		lbr.move(stopover);
		
		getLogger().info("Approaching...");
		PTP pick = ptp(revisedPick);
		pick.setJointVelocityRel(0.1);
		lbr.move(pick); 
		
		getLogger().info("Fetching rivet...");
		PTP pickUp = ptp(suction);
		pickUp.setJointVelocityRel(0.1);
		lbr.move(pickUp); 
		
		IMotionContainer positionHoldContainer = lbr.moveAsync((new PositionHold(impedanceControlMode, -1, null)));

		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok once rivet attached.", "OK");

		// As soon as the modal dialog returns, the motion container will be cancelled. This finishes the position hold. 
		positionHoldContainer.cancel();	
		
		getLogger().info("Got the rivet...");
		PTP liftUp = ptp(revisedPick);
		liftUp.setJointVelocityRel(0.1);
		lbr.move(liftUp); 
		
		ForceCondition pushAchieved = ForceCondition.createSpatialForceCondition(lbr.getFlange(),10);
		
		IRisingEdgeListener pushListener = new IRisingEdgeListener() {

			@Override
			public void onRisingEdge(ConditionObserver conditionObserver,
					Date time, int missedEvents) {
				getLogger().info("Rivet successfully installed!");
				PTP fetchNext = ptp(placeApproach);
				fetchNext.setJointVelocityRel(0.1);
				lbr.move(fetchNext);
				
			}
		};
		
		ConditionObserver pushObserver = getObserverManager().createConditionObserver(pushAchieved, 
				NotificationType.MissedEvents, pushListener);
		pushObserver.enable();
		
		getLogger().info("In transit...");
		PTP stepOne = ptp(waypoint);
		stepOne.setJointVelocityRel(0.1);
		lbr.move(stepOne);

		getLogger().info("Getting close...");
		PTP place = ptp(placeApproach);
		place.setJointVelocityRel(0.1);
		lbr.move(place.setMode(impedanceControlMode));
		
		PTP near = ptp(placeApproachCloser);
		near.setJointVelocityRel(0.1);
		lbr.move(near.setMode(impedanceControlMode));

		getLogger().info("Installing rivet");
		PTP install = ptp(pushInNew);
		install.setJointVelocityRel(0.1);
		lbr.move(install.setMode(impedanceControlMode));
		
		getLogger().info("Stop recording");
		blackBox.stopRecording();
	}
		
	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		Listener app = new Listener();
		app.runApplication();
	}
}
