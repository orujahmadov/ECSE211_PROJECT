import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.UltrasonicSensor;


/**
 * The Class SearchAndRescue.
 */
public class SearchAndRescue {



	/** The Constant LENGTH_OF_BOARD. */
	private static final int LENGTH_OF_BOARD = 12;

	/** The Constant WIDTH_OF_BOARD. */
	private static final int WIDTH_OF_BOARD = 12;


	//
	/** The Constant DATA_FOR_EACH_POINT. */
	private static final int DATA_FOR_EACH_POINT = 3;

	public static boolean isDone = false; //if true odo correction starts again



	/** The odometer. */
	private final Odometer odometer;

	/** The odometer correction. */
	private final OdometryCorrection odometerCorrection;

	/** The navigation. */
	private final Navigation navigation;

	/** The us front. */
	private final UltrasonicSensor usFront;

	/** The us back. */
	private final UltrasonicSensor usBack;

	/** The localization. */
	private final Localization localization;



	/** The start x. */
	private int startX;

	/** The start y. */
	private int startY;

	/** The start theta. */
	private int startTheta;
	
	int[] dropOffArea = new int[2];
	
	





	/** The pick up motor. */
	private final NXTRegulatedMotor leftMotor = Motor.A, rightMotor = Motor.B, pickUpMotor = Motor.C;



	/**
	 * Instantiates a new search and rescue.
	 *
	 * @param odo the odo
	 * @param nav the nav
	 * @param usFront the us front
	 * @param usBack the us back
	 * @param localization the localization
	 */
	public SearchAndRescue(Odometer odo, Navigation nav, UltrasonicSensor usFront,UltrasonicSensor usBack, Localization localization) {
		this.odometer = odo;
		this.usFront = usFront;
		this.usBack = usBack;
		this.navigation = nav;
		this.localization = localization;
		this.odometerCorrection = new OdometryCorrection(odometer);


	}

	//start the Search and Rescue mission
	/**
	 * Start mission.
	 */
	public void startMission(int mapNumber, int dropOffAreaX, int dropOffAreaY) {
		  dropOffArea[0] = dropOffAreaX ; 
		  dropOffArea[1] = dropOffAreaY ; 

		
		odometerCorrection.start();
		odometerCorrection.setPriority(Thread.MAX_PRIORITY);
		localization.startObservation(mapNumber);
		LCD.drawInt(localization.getStartX(),0,4);
		LCD.drawInt(localization.getStartY(),0,5);
		LCD.drawInt(localization.getStartTheta(),0,6);
		goToPickUpArea();
		navigation.turnTo(180);
		navigation.travelTo(15, 30);
		startPickingUp();
		navigation.travelTo(15,45);
		goToDropOffArea();
		dropOff();
		

		
		followPath(findShortestPath(dropOffArea[0],dropOffArea[1],1,2));
		navigation.turnTo(180);
        navigation.travelTo(15, 15);
        navigation.turnTo(225);
		startPickingUp();
		navigation.travelTo(15,15);
		navigation.travelTo(15,45);
		goToDropOffArea();
		dropOff();
		
		followPath(findShortestPath(dropOffArea[0],dropOffArea[1],1,2));
		navigation.turnTo(180);
        navigation.travelTo(15, 15);
        navigation.turnTo(270);
		startPickingUp();
		navigation.travelTo(15,15);
		navigation.travelTo(15,45);
		goToDropOffArea();
		dropOff();
		
		followPath(findShortestPath(dropOffArea[0],dropOffArea[1],1,2));
		navigation.turnTo(180);
        navigation.travelTo(15, 15);
        navigation.turnTo(225);
		startPickingUp();
		navigation.travelTo(15,15);
		navigation.travelTo(15,45);
		goToDropOffArea();
		dropOff();
		







	}





	/**
	 * Go to pick up area.
	 */
	public void goToPickUpArea(){
		followPath(findShortestPath(localization.getCurrentX(), localization.getCurrentY(), 1, 2));


	}


	/**
	 * Convert distance.
	 *
	 * @param radius the radius
	 * @param distance the distance
	 * @return the int
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}


	/**
	 * Find shortest path.
	 *
	 * @param startX the start x
	 * @param startY the start y
	 * @param destinationX the destination x
	 * @param destinationY the destination y
	 * @return the list
	 */
	public List findShortestPath(int startX,int startY, int destinationX, int destinationY){

		//another seperate map for navigation not to get confused
		//now we each point, we have 3 data
		//first is whether object is blocked or not
		//second is distance
		// tile checked for distance or not
		/** The map. */
	      final int[][][] map = new int[LENGTH_OF_BOARD][WIDTH_OF_BOARD][DATA_FOR_EACH_POINT];
	      
	        //first is blocked or not
	        //second is distance
	        //thirds checked or not
	        final int[][]   obstaclePositions = localization.getObstaclePositions();
	        final int[][]   pickUpAreaCoordinates = localization.getPickUpAreaCoordinates();
	 
	        int[] destination = {destinationX, destinationY};
	        int[] start = {startX,startY};
	 
	        map[destination[0]][destination[1]][2] = 1;
	          
	        ArrayList list = new ArrayList();
	          
	        int listStart=0;
	        list.add(destination[0]);
	        list.add(destination[1]);
	        int pointsAdded=0;
	        for(int i=0; i<obstaclePositions.length; i++){
	            int row = obstaclePositions[i][0];
	            int column = obstaclePositions[i][1];
	            map[row][column][0]=1;
	            map[row][column][2]=1;
	        }
	        
	        for(int i=0; i<pickUpAreaCoordinates.length; i++){
	            int row = pickUpAreaCoordinates[i][0];
	            int column = pickUpAreaCoordinates[i][1];
	            map[row][column][0]=1;
	            map[row][column][2]=1;
	        }
	       
	                for(int i=0; i<list.size();i+=2){
	                      
	                        for(int row=0; row<map.length; row++){
	                            for(int column=0; column<map.length; column++){
	                            	
	                    if((Math.abs(row-(Integer)list.get(i))==0 && Math.abs(column-(Integer)list.get(i+1))==1) ||
	                        Math.abs(row-(Integer)list.get(i))==1 && Math.abs(column-(Integer)list.get(i+1))==0){
	                       
	                    	if(map[row][column][2]!=1 && map[row][column][0]!=1){
	                      
	                    	map[row][column][1]=map[(Integer)list.get(i)][(Integer)list.get(i+1)][1]+1;
	                        map[row][column][2]=1;
	                        list.add(row);
	                        list.add(column);
	                        pointsAdded++;
	                        }
	                          
	                        }
	                    }
	                      
	                      
	                }
	            }
	      
	          
	        for(int i=0; i<obstaclePositions.length; i++){
	            map[obstaclePositions[i][0] ][ obstaclePositions[i][1]][1]=-100;
	        }
	        for(int i=0; i<pickUpAreaCoordinates.length; i++){
	            map[pickUpAreaCoordinates[i][0] ][ pickUpAreaCoordinates[i][1]][1]=-100;
	        }
	  
	        int distance= map[start[0]][start[1]][1];
	        int r = start[0];
	        int c = start[1];
	        ArrayList path = new ArrayList();
	          
	            while(map[r][c][1]!=0){
	                if(r-1>-1){
	                if (map[r][c][1]-map[r-1][c][1]==1){
	                    path.add(r-1);
	                    path.add(c);
	                    r=r-1;
	                }
	                }
	                if(r+1<map.length){
	                if (map[r][c][1]-map[r+1][c][1]==1){
	                    path.add(r+1);
	                    path.add(c);
	                    r=r+1;
	                }
	                }
	                if(c+1<map.length){
	                if (map[r][c][1]-map[r][c+1][1]==1){
	                    path.add(r);
	                    path.add(c+1);
	                    c=c+1;
	                }
	                }
	                if(c-1>-1){
	                if (map[r][c][1]-map[r][c-1][1]==1){
	                    path.add(r);
	                    path.add(c-1);
	                    c=c-1;
	                }
	                }
	                          
	                }
		return path;

	}

	//starts hardware to pick up blocks
	/**
	 * Start picking up.
	 */
	public void startPickingUp(){

		while(getDistanceToObject()>10){
			leftMotor.setSpeed(150);
			rightMotor.setSpeed(150);
			leftMotor.forward();
			rightMotor.forward();
		}


		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);



		double currentOrientation = Math.toDegrees(odometer.getTheta());
		int currentDistance = getDistanceToObject();


		navigation.turnTo(currentOrientation + 10);
		int rightDistance10 = getDistanceToObject();

		navigation.turnTo(currentOrientation + 20);
		int rightDistance20 = getDistanceToObject();

		navigation.turnTo(currentOrientation + 30);
		int rightDistance30 = getDistanceToObject();

		navigation.turnTo(currentOrientation + 40);
		int rightDistance40 = getDistanceToObject();

		navigation.turnTo(currentOrientation - 10);
		int leftDistance10 = getDistanceToObject();

		navigation.turnTo(currentOrientation - 20);
		int leftDistance20 = getDistanceToObject();

		navigation.turnTo(currentOrientation - 30);
		int leftDistance30 = getDistanceToObject();

		navigation.turnTo(currentOrientation - 40);
		int leftDistance40 = getDistanceToObject();

		if(shortestDistance(leftDistance40,leftDistance30,leftDistance20,leftDistance10, currentDistance,
				rightDistance10, rightDistance20, rightDistance30, rightDistance40) == currentDistance){
			navigation.turnTo(currentOrientation);
		}
		else if(shortestDistance(leftDistance40,leftDistance30,leftDistance20,leftDistance10, currentDistance,
				rightDistance10, rightDistance20, rightDistance30, rightDistance40) == leftDistance30){
			navigation.turnTo(currentOrientation-30);
		}

		else if(shortestDistance(leftDistance40,leftDistance30,leftDistance20,leftDistance10, currentDistance,
				rightDistance10, rightDistance20, rightDistance30, rightDistance40) == leftDistance20){
			navigation.turnTo(currentOrientation-20);
		}

		else if(shortestDistance(leftDistance40,leftDistance30,leftDistance20,leftDistance10, currentDistance,
				rightDistance10, rightDistance20, rightDistance30, rightDistance40) == leftDistance10){
			navigation.turnTo(currentOrientation-10);
		}

		else if(shortestDistance(leftDistance40,leftDistance30,leftDistance20,leftDistance10, currentDistance,
				rightDistance10, rightDistance20, rightDistance30, rightDistance40) == rightDistance40){
			navigation.turnTo(currentOrientation + 40);
		}

		else if(shortestDistance(leftDistance40,leftDistance30,leftDistance20,leftDistance10, currentDistance,
				rightDistance10, rightDistance20, rightDistance30, rightDistance40) == rightDistance30){
			navigation.turnTo(currentOrientation + 30);
		}

		else if(shortestDistance(leftDistance40,leftDistance30,leftDistance20,leftDistance10, currentDistance,
				rightDistance10, rightDistance20, rightDistance30, rightDistance40) == rightDistance20){
			navigation.turnTo(currentOrientation+20);
		}

		else if(shortestDistance(leftDistance40,leftDistance30,leftDistance20,leftDistance10, currentDistance,
				rightDistance10, rightDistance20, rightDistance30, rightDistance40) == rightDistance10){
			navigation.turnTo(currentOrientation+10);
		}
		
		


		navigation.move(shortestDistance(leftDistance40,leftDistance30,leftDistance20,leftDistance10, currentDistance,
				rightDistance10, rightDistance20, rightDistance30, rightDistance40)-20);





	
		

		pickUpMotor.flt();
        pickUpMotor.setSpeed(0);
        pickUpMotor.setSpeed(300);
        pickUpMotor.backward();
        pickUpMotor.rotate(-360);
        pickUpMotor.backward();
        pickUpMotor.rotate(-360);
        
 
 
 
 
        navigation.move(getDistanceToObject()-2);
 
 
 
 
        
        pickUpMotor.setSpeed(300);
        pickUpMotor.forward();
        pickUpMotor.rotate(240);
        pickUpMotor.forward();
        pickUpMotor.rotate(200);
        
 
 
 
 
        navigation.move(-10);
 
 
 
        pickUpMotor.setSpeed(300);
        pickUpMotor.rotate(360);
        pickUpMotor.forward();
        pickUpMotor.rotate(360);
		



		




		
		
		

	
		
		isDone = true;








	}
	
	/**
	 * Go to drop off area.
	 */
	public void goToDropOffArea(){
		followPath(findShortestPath(1, 2, dropOffArea[0], dropOffArea[1]));
		

	}
	


	public void dropOff(){
		navigation.move(-30);
		try{
		pickUpMotor.setSpeed(200);
		pickUpMotor.rotate(-400);
		Thread.sleep(1000);
		}catch(InterruptedException ex){
			
		}
		pickUpMotor.setSpeed(200);
		pickUpMotor.forward();
		pickUpMotor.rotate(360);

	}

	/**
	 * Follow path.
	 *
	 * @param pathPoints the path points
	 */
	public void followPath(List pathPoints){
		for (int i = 0; i<pathPoints.size(); i+=2){
			navigation.travelTo(((Integer)pathPoints.get(i))*30-15, ((Integer)pathPoints.get(i+1))*30-15);
		}

	}
	
//	private List combinePoints(List path, int X, int Y){
//		
//		
//		  ArrayList newPath = new ArrayList();
//		  int startX = X;
//          int startY = Y;
//
//          int i=0;
//           
//          while(i+3<path.size()){
//               
//              if((Integer)path.get(i)==startX){
//                  while(i+3<path.size() && (Integer)path.get(i)==startX){
//                      startY=(Integer) path.get(i+1);
//                      i+=2;
//                  }
//                  if(i+3>path.size()){
//                      newPath.add(startX);
//                      newPath.add(startY);
//                      startX=(Integer) path.get(i);
//                      startY=(Integer) path.get(i+1);
//                      newPath.add(startX);
//                      newPath.add(startY);
//                      break;
//                  }
//                  else{
//                  newPath.add(startX);
//                  newPath.add(startY);
//                  }
//              }
//               
//              else if((Integer)path.get(i+1)==startY){
//                  while(i+3<path.size() &&(Integer)path.get(i+1)==startY){
//                      startX=(Integer) path.get(i);
//                      i+=2;
//                  }
//                  if(i+3>path.size()){
//                      newPath.add(startX);
//                      newPath.add(startY);
//                      startX=(Integer) path.get(i);
//                      startY=(Integer) path.get(i+1);
//                      newPath.add(startX);
//                      newPath.add(startY);
//                      break;
//                  }
//                  newPath.add(startX);
//                  newPath.add(startY);
//               
//              }
//               
//          }
//          
//          return newPath;
//		
//	}

	/**
	 * Gets the distance to object.
	 *
	 * @return the distance to object
	 */
	public int getDistanceToObject() {

		int usData[] = new int[5];
		for (int i = 0; i < 4; i++) {
			usData[i] = usFront.getDistance(); }
		Arrays.sort(usData);
		
		return usData[2];




		

	}

	private int shortestDistance(int distance1, int distance2, int distance3 , int distance4 , int distance5,
			int distance6, int distance7, int distance8,  int distance9){
		int shortestDistance = 1000;
		int[] distances = new int[9];
		distances[0] = distance1;
		distances[1] = distance2;
		distances[2] = distance3;
		distances[3] = distance4;
		distances[4] = distance5;
		distances[5] = distance6;
		distances[6] = distance7;
		distances[7] = distance8;
		distances[8] = distance9;
		for(int i=0; i<distances.length; i++){
			if(distances[i]<shortestDistance){
				shortestDistance = distances[i];
			}
		}

		return shortestDistance;
	}





}