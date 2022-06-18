package artisynth.istar.hamidreza3.models.dynjaw;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import artisynth.core.inverse.InverseManager;
import artisynth.core.inverse.TrackingController;
import artisynth.core.inverse.InverseManager.ProbeID;
import artisynth.core.materials.LigamentAxialMaterial;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.rl.Log;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;

public class JawHamidreza extends JawLarynxDemo{

	HashMap<String,String> InsversMuscles = new HashMap<String,String>();
   
   public JawHamidreza() {
      super();
   }
   
   
   
   @Override
   public void build (String[] args) throws IOException {
       
	   super.build (args);
	   

	   myJawModel.rigidBodies().get("jaw").setFrameDamping (40);
	   myJawModel.rigidBodies().get("hyoid").setFrameDamping (100);
	   myJawModel.rigidBodies().get("thyroid").setFrameDamping (20);
	   myJawModel.rigidBodies().get("cricoid").setFrameDamping (20);
	   
	    
	   
	   Point3d leftPt = new Point3d(10.490128,-42.139055,17.073866);
	   FrameMarker leftInc = new FrameMarker(myJawModel.rigidBodies().get("jaw"), leftPt);
	   leftInc.setName("leftincisor");
	   myJawModel.addFrameMarker(leftInc);
	   RenderProps.setPointColor(leftInc, Color.GREEN);
	     
	       
	       
	      
	   Point3d rightPt = new Point3d(-34.478022,-12.191832,31.320838);
	   FrameMarker rightInc = new FrameMarker(myJawModel.rigidBodies().get("jaw"), rightPt);
	   rightInc.setName("rightincisor");
	   myJawModel.addFrameMarker(rightInc);
	   RenderProps.setPointColor(rightInc, Color.RED);
	      

	      
	   
	   
	    InsversMuscles.put("lip","Left Inferior Lateral Pterygoid");
		InsversMuscles.put("rip","Right Inferior Lateral Pterygoid");
		InsversMuscles.put("lsp","Lateral Pterygoid");
		InsversMuscles.put("rsp","Right Superior Lateral Pterygoid");
		InsversMuscles.put("lad","Left Anterior Digastric");
		InsversMuscles.put("rad","Right Anterior Digastric");
		InsversMuscles.put("lam","Left Mylohyoid");
		InsversMuscles.put("ram","Right Mylohyoid");
		//InsversMuscles.put("lpm","Left Posterior Mylohyoid");
		//InsversMuscles.put("rpm","Right Posterior Mylohyoid");
		InsversMuscles.put("lgh","Left Geniohyoid");
		InsversMuscles.put("rgh","Right Geniohyoid");
		InsversMuscles.put("lpd","Left Posterior Digastric");
		InsversMuscles.put("rpd","Right Posterior Digastric");
		InsversMuscles.put("lsteh","Left Sternohyoid");
		InsversMuscles.put("rsteh","Right Sternohyoid");
		InsversMuscles.put("lat","Left Anterior Temporal");
		InsversMuscles.put("rat","Right Anterior Temporal");
		InsversMuscles.put("lmt","Left Middle Temporal");
		InsversMuscles.put("rmt","Right Middle Temporal");
		InsversMuscles.put("lpt","Left Posterior Temporal");
		InsversMuscles.put("rpt","Right Posterior Temporal");
		InsversMuscles.put("lsm","Left Superficial Masseter");
		InsversMuscles.put("rsm","Right Superficial Masseter");
		InsversMuscles.put("lmp","Left Medial Pterygoid");
		InsversMuscles.put("rmp","Right Medial Pterygoid");
		InsversMuscles.put("ldm","Left Deep Masseter");
		InsversMuscles.put("rdm","Right Deep Masseter");


		
        inverseSetup();
        
        
        
        RigidBody jaw = myJawModel.rigidBodies().get("jaw");
		RigidBody skull = myJawModel.rigidBodies().get("maxilla");

		
        //add right condylar capsule frame markers
		
     	//createAddFrameMarker("rTmjOuterPosterior", new Point3d(-55.993489, -4.5712128, -10.36536), jaw);
     	//createAddFrameMarker("rTmjOuterAnterior", new Point3d(-56.049736, -8.4312215, -10.324311), jaw);
     	//createAddFrameMarker("rTmjInnerPosterior", new Point3d(-35.993489, -1.5712128, -9.36536), jaw);
     	//createAddFrameMarker("rTmjInnerAnterior", new Point3d(-36.049736, -5.4312215, -9.324311), jaw);
     	//createAddFrameMarker("rCapsulePosterior", new Point3d(-47.049736, 1.4312215, -5.324311), skull);
     	//createAddFrameMarker("rCapsuleAnterior", new Point3d(-48.569322, -18.338951, 1.7008584), skull);
		
		createAddFrameMarker("rCapsulePosterior", new Point3d(-50.384596, 42.218674, 77.844186), skull);
     	createAddFrameMarker("rTmjOuterPosterior", new Point3d(-56.247418, 34.854569, 74.090868), jaw);
     	createAddFrameMarker("rTmjOuterAnterior", new Point3d(-56.7857, 30.828629, 74.043919), jaw);
     	createAddFrameMarker("rTmjInnerPosterior", new Point3d(-44.999628, 35.083586, 73.880446), jaw);
     	createAddFrameMarker("rTmjInnerAnterior", new Point3d(-47.293817, 30.582638, 74.235523), jaw);
		
		
     	// add left condylar capsule frame markers
     	
     	//createAddFrameMarker("lTmjOuterPosterior", new Point3d(65.453532, -7.4198865, -6.9921023), jaw);
     	//createAddFrameMarker("lTmjOuterAnterior", new Point3d(65.936589, -11.156971, -6.4801041), jaw);
     	//createAddFrameMarker("lTmjInnerPosterior", new Point3d(43.93532, -3.4198865, -5.9921023), jaw);
     	//createAddFrameMarker("lTmjInnerAnterior", new Point3d(44.936589, -7.156971, -5.4801041), jaw);
     	//createAddFrameMarker("lCapsulePosterior", new Point3d(56.613578, -1.363587, -4.6223223), skull);
     	//createAddFrameMarker("lCapsuleAnterior", new Point3d(53.626018, -18.755689, 1.7161942), skull);
  
     	
     	createAddFrameMarker("lCapsulePosterior", new Point3d(50.384596, 42.218674, 77.844186), skull);
     	createAddFrameMarker("lTmjOuterPosterior", new Point3d(56.247418, 34.854569, 74.090868), jaw);
     	createAddFrameMarker("lTmjOuterAnterior", new Point3d(56.7857, 30.828629, 74.043919), jaw);
     	createAddFrameMarker("lTmjInnerPosterior", new Point3d(44.999628, 35.083586, 73.880446), jaw);
     	createAddFrameMarker("lTmjInnerAnterior", new Point3d(47.293817, 30.582638, 74.235523), jaw);
     	
     	setCondylarCapsule();
        
     	
     	
     	//createAddFrameMarker("rCapsulePosterior", new Point3d(-50.384596, 42.218674, 77.844186), skull);
     	//createAddFrameMarker("rTmjOuterPosterior", new Point3d(-56.247418, 34.854569, 74.090868), jaw);
     	//createAddFrameMarker("rTmjOuterAnterior", new Point3d(-56.7857, 30.828629, 74.043919), jaw);
     	//createAddFrameMarker("rTmjInnerPosterior", new Point3d(-44.999628, 35.083586, 73.880446), jaw);
     	//createAddFrameMarker("rTmjInnerAnterior", new Point3d(-47.293817, 30.582638, 74.235523), jaw);

   }
   
   
   public void inverseSetup() {
		TrackingController myTrackingController = new TrackingController(myJawModel, "incisor_disp");
		 
		for (Muscle muscle : myJawModel.myMuscles) {
		    for (String invMuscle: InsversMuscles.keySet()) {
		    	if (invMuscle.equals(muscle.getName())){
			    	myTrackingController.addExciter(muscle);	
		    	}	
		    	
		    }
			
		}
		
		myTrackingController.addMotionTarget(myJawModel.frameMarkers().get("lowerincisor"));
		myTrackingController.addMotionTarget(myJawModel.frameMarkers().get("leftincisor"));
		myTrackingController.addMotionTarget(myJawModel.frameMarkers().get("rightincisor"));

		myTrackingController.addL2RegularizationTerm(0.1327);
	    myTrackingController.setMaxExcitationJump (0.1);
	    //myTrackingController.setUseKKTFactorization(myEditableP);
		myTrackingController.setNormalizeH(true);
		myTrackingController.createProbesAndPanel (this);
	     
		InverseManager.setInputProbeData (
	    		this,
	            ProbeID.TARGET_POSITIONS,
	            new double[] { 0.0, -47.9584, 41.7642}, 
	            /*timestep=*/0.1);

	     addController(myTrackingController);
	     
	}
   
   protected void createAddFrameMarker(String name, Point3d position, Frame frame) {
		FrameMarker fm = new FrameMarker(frame, position);
		fm.setName(name);
		myJawModel.addFrameMarker(fm);
	}
  
  
  protected void setCondylarCapsule() {
		double slack = 7.5; // increase if jaw needs more flexibility
		LigamentAxialMaterial capsule_material = new LigamentAxialMaterial(2450, 0, 0);

		MultiPointSpring mpstring_r = new MultiPointSpring();
		mpstring_r.addPoint(myJawModel.frameMarkers().get("rCapsulePosterior"));
		mpstring_r.addPoint(myJawModel.frameMarkers().get("rTmjOuterPosterior"));
		mpstring_r.addPoint(myJawModel.frameMarkers().get("rTmjOuterAnterior"));
		// mpstring_r.addPoint(frameMarkers().get("rCapsuleAnterior")); // removed
		mpstring_r.addPoint(myJawModel.frameMarkers().get("rTmjInnerAnterior"));
		mpstring_r.addPoint(myJawModel.frameMarkers().get("rTmjInnerPosterior"));
		mpstring_r.addPoint(myJawModel.frameMarkers().get("rCapsulePosterior"));
		mpstring_r.setMaterial(capsule_material);
		mpstring_r.getRenderProps().setLineStyle(LineStyle.LINE);
		mpstring_r.setRestLength(mpstring_r.getLength() + slack);
		mpstring_r.getRenderProps().setLineColor(Color.GREEN);
		mpstring_r.getRenderProps().setLineStyle(LineStyle.CYLINDER);
		mpstring_r.getRenderProps().setLineRadius(0.75);
		myJawModel.addMultiPointSpring(mpstring_r);

		MultiPointSpring mpstring_l = new MultiPointSpring();
		mpstring_l.addPoint(myJawModel.frameMarkers().get("lCapsulePosterior"));
		mpstring_l.addPoint(myJawModel.frameMarkers().get("lTmjOuterPosterior"));
		mpstring_l.addPoint(myJawModel.frameMarkers().get("lTmjOuterAnterior"));
		// mpstring_l.addPoint(frameMarkers().get("lCapsuleAnterior")); // removed
		mpstring_l.addPoint(myJawModel.frameMarkers().get("lTmjInnerAnterior"));
		mpstring_l.addPoint(myJawModel.frameMarkers().get("lTmjInnerPosterior"));
		mpstring_l.addPoint(myJawModel.frameMarkers().get("lCapsulePosterior"));
		mpstring_l.setMaterial(capsule_material);
		mpstring_l.getRenderProps().setLineStyle(LineStyle.LINE);
		mpstring_l.setRestLength(mpstring_l.getLength() + slack);
		mpstring_l.getRenderProps().setLineColor(Color.GREEN);
		mpstring_l.getRenderProps().setLineStyle(LineStyle.CYLINDER);
		mpstring_l.getRenderProps().setLineRadius(0.75);
		myJawModel.addMultiPointSpring(mpstring_l);

		
	}

   
   
}

