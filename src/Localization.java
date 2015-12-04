import java.util.ArrayList;

/**
 * this class localizes the robot
 * @author Oruj Ahmadov, Sangmoon Hwang
 *
 */

import java.util.Arrays;

import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

// TODO: Auto-generated Javadoc
/**
 * The Class Localization.
 */
public class Localization {


	/** The Constant LENGTH_OF_BOARD. */
	private static final int LENGTH_OF_BOARD = 12;

	/** The Constant WIDTH_OF_BOARD. */
	private static final int WIDTH_OF_BOARD = 12;

	//basically for each point, we need 8 data
	//obstacle values for 4 cardinal directions
	// north, west, south and east accordingly
	//and boolean value indicating whether it is potential
	//starting positions or not. for the beginning, it will 
	//all set to 1, but as we do observation, we will eliminate them by setting to 0
	//e.g point -15,-15, with obstacle on right will be respresented as
	// {0,1,1,1,1,1,1,1}
	/** The Constant DATA_FOR_EACH_POINT. */
	private static final int DATA_FOR_EACH_POINT = 8;


	/** The odo. */
	private final Odometer odometer;

	/** The nav. */
	private final Navigation navigation;

	/** The us front. */
	private final UltrasonicSensor usFront;

	/** The us back. */
	private final UltrasonicSensor usBack;

	/** The observation count. */
	private int observationCount = 0;

	/** The potential positions. */
	private int potentialPositions;


	/** The rotations count. */
	private int rotationsCount = 0;

	/** The obstacle lists. */
	private ArrayList obstacleLists = new ArrayList();

	/** The theta. */
	private double theta = -90;

	/** The front distance. */
	int frontDistance;

	/** The back distance. */
	int backDistance;

	/** The start x. */
	private int startX;

	/** The start y. */
	private int startY;

	/** The start theta. */
	private int startTheta;

	/** The current x. */
	private int currentX;

	private double errorX;

	private double errorY;

	/** The current y. */
	private int currentY;


	// using list to keep record of movements of robots
	// so that those movements can be applied to potential
	// positions as well when we check them for obstacle for another time

	/** The movements list. */
	private final ArrayList movementsList = new ArrayList();

	/** The pick up motor. */
	private final NXTRegulatedMotor leftMotor = Motor.A, rightMotor = Motor.B;

	// 3 dimensional array is used to build fixed map
	// the map is built like chess model
	//so each row corresponds the points vertically
	// and each column corresponds to points horizontally
	//to find the X and Y values of any point from map
	// we have
	//x = -15 + 30*ROW
	//y = -15 + 30*COLUMN
	/** The map. */
	final int[][][] map = new int[LENGTH_OF_BOARD][WIDTH_OF_BOARD][DATA_FOR_EACH_POINT];

	/** The obstacle positions */
	private int[][]   obstaclePositions;



	//the obstacle positions for 6 different maps
	//once we get maps, we will file these arrays accordingly

	/** The obstacle positions map1. */
	final int[][]   obstaclePositionsMap1 = { {0,6}, {0,9}, {1,3}, {2,2}, {2,11}, 
			                                  {3,5}, {3,10}, {4,4}, {4,7}, {5,0}, 
			                                  {5,2}, {6,3}, {7,7}, {8,0}, {8,1}, 
			                                  {8,6}, {9,4}, {9,7}, {9,9}, {10,2}, 
			                                  {10,6},{11,1}};

	/** The obstacle positions map2. */
	final int[][]   obstaclePositionsMap2 = {{0,4},{0,5},{3,2},{3,5},{3,6},
			                                 {3,9},{4,5},{4,9},{4,10},{5,5},
			                                 {6,8},{6,9},{7,0},{8,4},{8,5},
			                                 {8,10},{9,1},{9,11},{10,7},{11,2},
			                                 {11,5},{11,8}};

	/** The obstacle positions map3. */
	final int[][]   obstaclePositionsMap3 = {{0,3},{0,8},{2,3},{2,6},{3,2},
                                             {3,4},{3,10},{4,1},{4,5},{4,6},
                                             {4,7},{6,6},{6,8},{7,0},{7,11},
                                             {9,6},{10,0},{10,3},{10,4},{10,9},
                                             {11,2},{11,11}};
	
	/** The obstacle positions map4. */
	final int[][]   obstaclePositionsMap4 = {{0,2},{0,3},{0,4},{0,8},{1,4},
                                             {2,0},{2,5},{2,10},{3,2},{3,9},
                                             {3,11},{4,3},{5,10},{6,4},{8,4},
                                             {8,8},{9,0},{9,4},{9,6},{11,1},
                                             {11,5},{11,10}};
	
	/** The obstacle positions map5. */
	final int[][]   obstaclePositionsMap5 = {{0,3},{0,5},{0,8},{3,1},{3,3},
                                             {3,10},{4,3},{4,6},{4,7},{4,10},
                                             {5,8},{6,9},{7,6},{7,7},{8,1},
                                             {8,5},{8,10},{9,2},{9,6},{10,10},
                                             {11,2},{11,9}};
	
	/** The obstacle positions map6. */
	final int[][]   obstaclePositionsMap6 = {{0,10},{1,5},{2,3},{3,2},{3,10},
                                             {4,2},{4,7},{5,2},{5,4},{5,6},
                                             {5,11},{6,5},{6,9},{7,6},{7,8},
                                             {8,2},{8,10},{9,0},{9,4},{9,8},
                                             {10,5},{10,10}};
	



	private int[][] pickUpAreaCoordinates= {{0,0},{0,1},{1,0},{1,1}};









	/**
	 * constructor.
	 *
	 * @param odo the odo
	 * @param nav the nav
	 * @param usFront the us front
	 * @param usBack the us back
	 */
	public Localization(Odometer odo, Navigation nav, UltrasonicSensor usFront, UltrasonicSensor usBack) {
		this.odometer = odo;
		this.usFront = usFront;
		this.usBack = usBack;
		this.navigation = nav;


	}

	//method start observing to find where it is
	/**
	 * Start observation.
	 */
	public void startObservation(int mapNumber) {



		//int mapNumber = Bluetooth.getMapValue();
		//find corresponding map value

		if(mapNumber==1){
			obstaclePositions = obstaclePositionsMap1;
		}
		else if(mapNumber==2){
			obstaclePositions = obstaclePositionsMap2;
		}

		else if(mapNumber==3){
			obstaclePositions = obstaclePositionsMap3;
		}
		else if(mapNumber==4){
			obstaclePositions = obstaclePositionsMap4;
		}

		else if(mapNumber==5){
			obstaclePositions = obstaclePositionsMap5;
		}
		
		else if(mapNumber==6){
			obstaclePositions = obstaclePositionsMap6;
		}

	


		potentialPositions = map.length*map.length*4 - obstaclePositions.length*4 - pickUpAreaCoordinates.length*4;

		//here we basically set up the MAP
		updateMap(obstaclePositions);



		frontDistance = getFrontDistance();
		backDistance = getBackDistance();
		int obstacleValue = getObstacleForOneTile(frontDistance, backDistance);
		doObservation(obstacleValue);

		while (potentialPositions >= 2) {



			// if both of sensors see an obstacle , we turn for -90 degree
			//NOTE: getObstacle value for one tile return values for both back and front us
			//(return of method)/10 gives front US , (return%10) gives back US
			if (obstacleValue/10 == 1 && obstacleValue%10 ==1) {


				navigation.turnTo(theta);
				movementsList.add(1);
				rotationsCount++;

				theta = theta - 90;

				frontDistance = getFrontDistance();
				backDistance = getBackDistance();
				obstacleValue = getObstacleForOneTile(frontDistance, backDistance);
				doObservation(obstacleValue);
				if(potentialPositions<2){
					break;
				}

				if(obstacleValue/10==0){
					doFurtherObservation(getObstacleForTwoTile(frontDistance, backDistance)/10, 0);
				}
				if(obstacleValue%10==0){
					doFurtherObservation(getObstacleForTwoTile(frontDistance, backDistance)%10, 1);
				}

				if(potentialPositions<2){
					break;
				}

				if(obstacleValue/10!=1){
					navigation.move(30);
					movementsList.add(2);
				}
				else if(obstacleValue%10!=1){
					navigation.move(-30);
					movementsList.add(3);
				}
				else if(obstacleValue / 10==1 && obstacleValue % 10==1){
					navigation.turnTo(theta);
					movementsList.add(1);
					rotationsCount++;

					theta = theta - 90;
					if(getObstacleForOneTile(getFrontDistance(), getBackDistance())%10==0){
						navigation.move(-30);
						movementsList.add(3);
					}
					else{ 
						navigation.move(30);
						movementsList.add(2);
					}

				}






			}

			//if no sensor sees any obstacle, do further observation with both US
			//without moving the robot
			else if (obstacleValue / 10 == 0 && obstacleValue % 10 ==0) {

				doFurtherObservation(getObstacleForTwoTile(frontDistance, backDistance)/10, 0);
				doFurtherObservation(getObstacleForTwoTile(frontDistance, backDistance)%10, 1);


				navigation.turnTo(theta);
				movementsList.add(1);
				rotationsCount++;

				theta = theta - 90;

				frontDistance = getFrontDistance();
				backDistance = getBackDistance();
				obstacleValue = getObstacleForOneTile(frontDistance, backDistance);
				doObservation(obstacleValue);
				if(potentialPositions<2){
					break;
				}

				if(obstacleValue/10==0){
					doFurtherObservation(getObstacleForTwoTile(frontDistance, backDistance)/10, 0);
				}
				if(obstacleValue%10==0){
					doFurtherObservation(getObstacleForTwoTile(frontDistance, backDistance)%10, 1);
				}

				if(potentialPositions<2){
					break;
				}
				if(obstacleValue/10==0){
					navigation.move(30);
					movementsList.add(2);
				}
				else if(obstacleValue % 10==0){
					navigation.move(-30);
					movementsList.add(3);
				}

				else if(obstacleValue / 10==1 && obstacleValue % 10==1){
					navigation.turnTo(theta);
					movementsList.add(1);
					rotationsCount++;

					theta = theta - 90;
					if(getObstacleForOneTile(getFrontDistance(), getBackDistance())%10==0){
						navigation.move(-30);
						movementsList.add(3);
					}
					else{
						navigation.move(30);
						movementsList.add(2);
					}

				}


			}

			//if only one of US does not see an obstacle, do further Observation with that
			//US, then turn for -90
			else if(obstacleValue/10 == 1 || obstacleValue%10 ==1){

				if(obstacleValue/10==0){
					doFurtherObservation(getObstacleForTwoTile(frontDistance, backDistance)/10, 0);
				}
				if(obstacleValue % 10==0){
					doFurtherObservation(getObstacleForTwoTile(frontDistance, backDistance)%10, 1);
				}

				if(potentialPositions<2){
					break;
				}


				navigation.turnTo(theta);
				movementsList.add(1);
				rotationsCount++;

				theta = theta - 90;

				frontDistance = getFrontDistance();
				backDistance = getBackDistance();
				obstacleValue = getObstacleForOneTile(frontDistance, backDistance);
				doObservation(obstacleValue);

				if(potentialPositions<2){
					break;
				}

				if(obstacleValue / 10==0){
					doFurtherObservation(getObstacleForTwoTile(frontDistance, backDistance)/10, 0);
				}
				if(obstacleValue % 10==0){
					doFurtherObservation(getObstacleForTwoTile(frontDistance, backDistance)%10, 1);
				}

				if(potentialPositions<2){
					break;
				}

				if(obstacleValue/ 10 == 0){
					navigation.move(30);
					movementsList.add(2);
				}
				else if(obstacleValue%10 == 0){
					navigation.move(-30);
					movementsList.add(3);
				}

				else if(obstacleValue / 10==1 && obstacleValue % 10==1){
					navigation.turnTo(theta);
					movementsList.add(1);
					rotationsCount++;

					theta = theta - 90;
					if(getObstacleForOneTile(getFrontDistance(), getBackDistance())%10==0){
						navigation.move(-30);
						movementsList.add(3);
					}
					else{
						navigation.move(30);
						movementsList.add(2);
					}

				}




			}


			frontDistance = getFrontDistance();
			backDistance = getBackDistance();
			obstacleValue = getObstacleForOneTile(frontDistance, backDistance);
			doObservation(obstacleValue);




		}

		Sound.beep();

		for (int row = 0; row < map.length; row++) {
			for (int column = 0; column < map.length; column++) {
				for(int direction =4; direction <8; direction++ ){
					// now that we have only one potential position left,
					// which has boolean potential 1, we correct Odometer accordingly
					if (map[row][column][direction] == 1) {
						startX =  row*30-15;
						startY = column*30-15;
						startTheta = (4-direction)*90+360;

						currentX = applyMovements(row, column, direction-4)[0];
						currentY = applyMovements(row, column, direction-4)[1];
						
						odometer.setX((applyMovements(row, column, direction-4)[0])*30-15);
						odometer.setY((applyMovements(row, column, direction-4)[1])*30-15);

						if(((direction-4 + rotationsCount)%4)*(-90)==0){
							odometer.setTheta(0);
						}
						else{
							odometer.setTheta( ((((direction-4+rotationsCount)%4)*(-90)+360)*Math.PI)/180 );
						}
					}

				}

			}

		}







	}

	// a separate method to make an observation
	// the method compares obstacle value with potential positions
	// value and make appropriate eliminations
	// also potential positions are checked according to
	// movements and rotations
	/**
	 * Do observation.
	 *
	 * @param obstacleValues the obstacle values
	 */
	private void doObservation(int obstacleValues) {


		int frontUSObstacleValue = obstacleValues / 10;
		int backUSObstacleValue  = obstacleValues % 10;

		for (int row = 0; row < map.length; row++) {
			for (int column = 0; column < map.length; column++) {
				for(int booleanPotential = 4; booleanPotential <8; booleanPotential++ ){

					// only check potential starting positions
					if (map[row][column][booleanPotential] != 0) {

						// below statement apply movements/rotations
						// to potential positions,then check for obstacle

						if (map[ (applyMovements(row, column, booleanPotential-4))[0] ][ (applyMovements(row, column, booleanPotential-4))[1] ][ (booleanPotential - 4 + rotationsCount) % 4 ]  != frontUSObstacleValue ||
								map[ (applyMovements(row, column, booleanPotential-4))[0] ][ (applyMovements(row, column, booleanPotential-4))[1] ][ (booleanPotential - 2 + rotationsCount) % 4] != backUSObstacleValue) {

							// eliminate positions that don't pass obstacle test
							map[row][column][booleanPotential] = 0;
							potentialPositions--;
						}


					}


				}

			}
		}

		observationCount+=2;
	}

	// a separate method to make an observation
	// the method compares obstacle value with potential positions
	// value and make appropriate eliminations
	// also potential positions are checked according to
	// movements and rotations
	//but unlike the doObservation method, method below 
	//visually move arrows one tile and then check for obstacle value
	//sensor Value, 0 for front, 1 for back
	/**
	 * Do further observation.
	 *
	 * @param obstacleValue the obstacle value
	 * @param sensorValue the sensor value
	 */
	private void doFurtherObservation(int obstacleValue, int sensorValue){
		//if it is front , we check 1 further tile, if it is back, one tile further back

		for (int row = 0; row < map.length; row++) {
			for (int column = 0; column < map.length; column++) {
				for(int booleanPotential = 4; booleanPotential <8; booleanPotential++ ){


					// only check potential starting positions
					if (map[row][column][booleanPotential] != 0) {

						int direction = (booleanPotential - 4 + rotationsCount)%4;
						//IF FRONT SENSOR FURTHER OBSERVATIONS IS BEING DONE
						if(sensorValue==0){
							if(direction==0){

								if (map[ (applyMovements(row, column, booleanPotential-4))[0] ][ (applyMovements(row, column, booleanPotential-4))[1] + 1 ][ (booleanPotential - 4 + rotationsCount)%4 ]  != obstacleValue) {

									// eliminate positions that don't pass obstacle test
									map[row][column][booleanPotential] = 0;
									potentialPositions--;
								}
							}
							else if(direction==1){

								if (map[ (applyMovements(row, column, booleanPotential-4))[0] - 1 ][ (applyMovements(row, column, booleanPotential-4))[1] ][ (booleanPotential - 4 + rotationsCount)%4 ]  != obstacleValue) {

									// eliminate positions that don't pass obstacle test
									map[row][column][booleanPotential] = 0;
									potentialPositions--;
								}

							}
							else if(direction==2){

								if (map[(applyMovements(row, column, booleanPotential-4))[0] ][ (applyMovements(row, column, booleanPotential-4))[1] - 1][(booleanPotential - 4 + rotationsCount)%4 ]  != obstacleValue) {

									// eliminate positions that don't pass obstacle test
									map[row][column][booleanPotential] = 0;
									potentialPositions--;
								}

							}

							else if(direction==3){

								if (map[(applyMovements(row, column, booleanPotential-4))[0] + 1][(applyMovements(row, column, booleanPotential-4))[1]][(booleanPotential - 4 + rotationsCount)%4 ]  != obstacleValue) {

									// eliminate positions that don't pass obstacle test
									map[row][column][booleanPotential] = 0;
									potentialPositions--;
								}

							}


						}




						//IF BACK SENSOR FURTHER OBSERVATIONS IS BEING DONE
						else if(sensorValue==1){

							if(direction==0){

								if (map[(applyMovements(row, column, booleanPotential-4))[0]][(applyMovements(row, column, booleanPotential-4))[1] - 1][((booleanPotential - 2)%4 + rotationsCount)%4 ]  != obstacleValue) {

									// eliminate positions that don't pass obstacle test
									map[row][column][booleanPotential] = 0;
									potentialPositions--;
								}
							}
							else if(direction==1){

								if (map[(applyMovements(row, column, booleanPotential-4))[0] + 1][(applyMovements(row, column, booleanPotential-4))[1]][((booleanPotential - 2)%4 + rotationsCount)%4 ]  != obstacleValue) {

									// eliminate positions that don't pass obstacle test
									map[row][column][booleanPotential] = 0;
									potentialPositions--;
								}

							}
							else if(direction==2){

								if (map[(applyMovements(row, column, booleanPotential-4))[0]][(applyMovements(row, column, booleanPotential-4))[1] + 1][((booleanPotential - 2)%4 + rotationsCount)%4 ]  != obstacleValue) {

									// eliminate positions that don't pass obstacle test
									map[row][column][booleanPotential] = 0;
									potentialPositions--;
								}

							}

							else if(direction==3){

								if (map[(applyMovements(row, column, booleanPotential-4))[0] - 1][(applyMovements(row, column, booleanPotential-4))[1]][((booleanPotential - 2)%4 + rotationsCount)%4 ]  != obstacleValue) {

									// eliminate positions that don't pass obstacle test
									map[row][column][booleanPotential] = 0;
									potentialPositions--;
								}

							}

						}


					}



				}

			}
		}

		observationCount++;

	}

	// the method checks whether robot facing
	// an obstacle or not in the range of one tile
	/**
	 * Gets the obstacle for one tile.
	 *
	 * @param sensorReadingFront the sensor reading front
	 * @param sensorReadingBack the sensor reading back
	 * @return the obstacle for one tile
	 */
	private int getObstacleForOneTile(int sensorReadingFront, int sensorReadingBack) {

		int frontUS = 0;
		int backUS  = 0;




		if (sensorReadingFront < 15) {
			frontUS =  1;
		}
		if (sensorReadingBack < 22) {
			backUS = 1;
		}


		return frontUS * 10 + backUS * 1;

	}

	// the method checks whether robot has
	// an obstacle or not in the range of two tiles
	/**
	 * Gets the obstacle for two tile.
	 *
	 * @param sensorReadingFront the sensor reading front
	 * @param sensorReadingBack the sensor reading back
	 * @return the obstacle for two tile
	 */
	private int getObstacleForTwoTile(int sensorReadingFront, int sensorReadingBack) {

		int frontUS = 0;
		int backUS  = 0;


		if (sensorReadingFront < 45) {
			frontUS =  1;
		}
		if (sensorReadingBack < 55) {
			backUS = 1;
		}



		return frontUS * 10 + backUS * 1;


	}

	/**
	 * Gets the front distance.
	 *
	 * @return the front distance
	 */
	private int getFrontDistance(){
		int usData[] = new int[5];
		for (int i = 0; i < 4; i++) {
			usData[i] = usFront.getDistance(); 
			}
		Arrays.sort(usData); return usData[2];

	}

	/**
	 * Gets the back distance.
	 *
	 * @return the back distance
	 */
	private int getBackDistance(){
		int usData[] = new int[5];
		for (int i = 0; i < 4; i++) {
			usData[i] = usBack.getDistance(); 
			}
		Arrays.sort(usData); return usData[2];

	}






	// once we have the list of movements
	// we can apply them any potential position
	// to check for obstacle test

	/**
	 * Apply movements.
	 *
	 * @param Row the row
	 * @param Column the column
	 * @param Direction the direction
	 * @return the int[]
	 */
	private int[] applyMovements(int Row, int Column, int Direction) {
		int[] returnData= new int[2];

		for (int i = 0; i < movementsList.size(); i++) {


			//rotation
			if ((Integer) (movementsList.get(i)) == 1) {

				Direction = (Direction+1)%4;

			}


			//if forward movement
			if ((Integer) (movementsList.get(i)) == 2) {
				if(Direction==3){
					Row=Row+1;
				}
				else if(Direction==0){
					Column=Column+1;
				}
				else if(Direction==1){
					Row=Row-1;
				}
				else if(Direction==2){
					Column=Column-1;
				}

			}

			//if backwards, basically opposite of what happens in forward movement
			if ((Integer) (movementsList.get(i)) == 3) {

				if(Direction==3){
					Row=Row-1;
				}
				else if(Direction==0){
					Column=Column-1;
				}
				else if(Direction==1){
					Row=Row+1;
				}
				else if(Direction==2){
					Column=Column+1;
				}

			}

		}

		returnData[0] = Row;
		returnData[1] = Column;
		return returnData;

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



	//the method below will update obstacle values
	//inside the map after getting which map
	//is being used via Bluetooth

	/**
	 * Update map.
	 *
	 * @param competitionMap the competition map
	 */
	private void updateMap(int competitionMap[][]){

		map[0][0][1]= 1; 
		map[0][0][2]= 1;
		map[0][map.length-1][0]= 1;
		map[0][map.length-1][1]= 1; 
		map[map.length-1][0][2]= 1; 
		map[map.length-1][0][3]= 1;
		map[map.length-1][map.length-1][0]= 1;
		map[map.length-1][map.length-1][3]= 1;

		//here we update obstacle Values for those arrow that face to walls
		for(int row=0; row<map.length ;row++){

			map[row][0][2]=1;

			map[row][map.length-1][0]=1;
		}
		for(int column = 0; column<map.length; column++){


			map[0][column][1]=1;

			map[map.length-1][column][3]=1;

		}


		//make all positions potential start
		for(int row=0; row<map.length; row++){
			for(int column=0; column<map.length; column++){
				map[row][column][4]=1;
				map[row][column][5]=1;
				map[row][column][6]=1;
				map[row][column][7]=1;


			}
		}




		//pick up area cannot be starting position
		for(int i=0; i<pickUpAreaCoordinates.length; i++){
			int row = pickUpAreaCoordinates[i][0];
			int column = pickUpAreaCoordinates[i][1];
			map[row][column][4]=0;
			map[row][column][5]=0;
			map[row][column][6]=0;
			map[row][column][7]=0;
		}

		//this loop however updates obstacle Values for those arrows that face to obstacle      

		for(int row = 0; row < competitionMap.length; row++){

			int i = competitionMap[row][0];
			int j = competitionMap[row][1];

			//below statements make obstacle positions
			//non-potential starting positions
			map[i][j][4]=0;
			map[i][j][5]=0;
			map[i][j][6]=0;
			map[i][j][7]=0;
		}
		for(int row =0; row < competitionMap.length; row++){
			int i = competitionMap[row][0];
			int j = competitionMap[row][1];
			if(j-1>=0){
				map[i][j-1][0]=1;
			}
			if(i+1<map.length){
				map[i+1][j][1]=1;
			}
			if(j+1<map.length){
				map[i][j+1][2]=1;
			}
			if(i-1>=0){
				map[i-1][j][3]=1;
			}
		}





	}





	//return X position of start point
	/**
	 * Gets the start x.
	 *
	 * @return the start x
	 */
	public int getStartX(){
		return startX;
	}

	//return Y position of start point
	/**
	 * Gets the start y.
	 *
	 * @return the start y
	 */
	public int getStartY(){
		return startY;
	}

	//return start Prientation of Robot
	/**
	 * Gets the start theta.
	 *
	 * @return the start theta
	 */
	public int getStartTheta(){
		return startTheta;
	}

	//method returns current X in coded values (0-11)
	/**
	 * Gets the current x.
	 *
	 * @return the current x
	 */
	public int getCurrentX(){
		return currentX;
	}

	//method returns current Y in coded values (0-11)
	/**
	 * Gets the current y.
	 *
	 * @return the current y
	 */
	public int getCurrentY(){
		return currentY;
	}



	/**
	 * Gets the obstacle positions.
	 *
	 * @return the obstacle positions
	 */
	public int[][] getObstaclePositions(){
		return obstaclePositions;

	}

	/**
	 * Gets the pick up area coordinates.
	 *
	 * @return the pick up area coordinates
	 */
	public int[][] getPickUpAreaCoordinates(){
		return pickUpAreaCoordinates;

	}



}