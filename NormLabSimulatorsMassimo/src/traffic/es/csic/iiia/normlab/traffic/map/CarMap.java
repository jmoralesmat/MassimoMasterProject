package es.csic.iiia.normlab.traffic.map;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import repast.simphony.context.Context;
import repast.simphony.space.grid.Grid;
import repast.simphony.space.grid.GridPoint;
import es.csic.iiia.normlab.traffic.TrafficSimulator;
import es.csic.iiia.normlab.traffic.agent.Car;
import es.csic.iiia.normlab.traffic.agent.Collision;
import es.csic.iiia.normlab.traffic.agent.TrafficElement;
import es.csic.iiia.normlab.traffic.car.CarPosition;
import es.csic.iiia.normlab.traffic.car.CarAction;
import es.csic.iiia.normlab.traffic.car.context.TrafficStateCodifier;
import es.csic.iiia.normlab.traffic.factory.CarContextFactory;
import es.csic.iiia.normlab.traffic.factory.TrafficFactFactory;
import es.csic.iiia.normlab.traffic.normsynthesis.TrafficNormSynthesisSettings;
import es.csic.iiia.normlab.traffic.utils.Direction;
import es.csic.iiia.normlab.traffic.utils.Turn;
import es.csic.iiia.normlab.traffic.utils.Utilities;
import es.csic.iiia.nsm.agent.language.PredicatesDomains;
import es.csic.iiia.nsm.norm.Norm;

/**
 * CarMap - Meta information management 
 * 
 * @author "Javier Morales (jmorales@iiia.csic.es)"
 *
 */
public class CarMap extends TrafficMatrix {
	
	//---------------------------------------------------------------------------
	// Attributes 
	//---------------------------------------------------------------------------
	
	private PredicatesDomains predDomains;
	private CarContextFactory carContextFactory;
	private TrafficFactFactory factFactory;
	
	private List<Car> allCars, travelingCars, carsToRemove;
	private LinkedList<Car> availableCars;
	private List<Collision> collisions, collisionsToRemove;
	private Map<String, GridPoint> positionsToCheck;
	
	private Context<TrafficElement> context = null;
	private Grid<TrafficElement> map = null;

	private int xDim = 0;
	private int yDim = 0;
	private long lastCarEmitTick = 1l;
	private int lowerLane = 0;
	private int upperLane = 0;
	private int leftLane = 0;
	private int rightLane = 0;

	/**
	 * Traffic view dimensions
	 */
	private int startRow, stopRow, startCol, stopCol;
		
	//---------------------------------------------------------------------------
	// Constructors 
	//---------------------------------------------------------------------------

	/**
	 * Constructor
	 *  
	 * @param context
	 * @param grid
	 * @param normLayer
	 */
	public CarMap(Context<TrafficElement> context, Grid<TrafficElement> map,
			PredicatesDomains predDomains, CarContextFactory carContextFactory,
			TrafficFactFactory factFactory) {
		
		super(map.getDimensions().getHeight(), map.getDimensions().getWidth());
		
		this.predDomains = predDomains;
		this.carContextFactory = carContextFactory;
		this.factFactory = factFactory;
		
		this.xDim = map.getDimensions().getWidth();
		this.yDim = map.getDimensions().getHeight();
		this.context = context;
		this.map = map;

		this.availableCars = new LinkedList<Car>();
		this.travelingCars = new ArrayList<Car>();
		this.allCars = new ArrayList<Car>();
		this.carsToRemove = new ArrayList<Car>();
		this.collisions = new ArrayList<Collision>();
		this.collisionsToRemove = new ArrayList<Collision>();
		this.positionsToCheck = new HashMap<String,GridPoint>();
		
		this.leftLane =  (int)(Math.floor(0.5*xDim))-1;
		this.rightLane = leftLane+2;
		this.lowerLane = (int)(Math.floor(0.5*yDim))-1;
		this.upperLane = lowerLane+2;
		
		this.startRow = 0;
		this.stopRow = yDim-1;
		this.startCol = 0;
		this.stopCol = xDim-1;
		
		this.generateCars();
	}

	//----------------------------------------------------------
	// Methods 
	//----------------------------------------------------------

	/**
	 * Generates the collection of cars to use
	 */
	public void generateCars() {
		Car car;

		for(short i=1; i<100; i++)
		{
			car = new Car(i, true, predDomains, carContextFactory, factFactory);
			allCars.add(car);
			availableCars.add(car);
		}
	}

	//------------------------------------------------------------------
	// 
	//------------------------------------------------------------------

	/**
	 * Makes all the works that must be done by the map in a step
	 * 
	 * @return
	 */
	public void step()
	{
		this.removeCollisions();
		this.moveCars();
		this.emitCars();  // Mete nuevos coches
		this.codify();
		
		for(Car car : this.travelingCars)
			car.perceiveAndReason();
	}

	/**
	 * 
	 */
	private void codify()
	{
		this.clear();
		String codState;
		
		// Clear previous information
		this.clear();
		
		for(int row=startRow; row<=stopRow; row++) {
			for(int col=startCol; col<=stopCol; col++) {
  			TrafficElement elem = this.getElement(row, col);
  			
  				// Create binary description and add it to the position
  				codState = TrafficStateCodifier.codify(elem);  				
  				this.set(row, col, codState);
			}
		}
	}
	
	/**
	 * Executes step method for each car and adds them to their new position
	 */
	private void moveCars()
	{
		this.carsToRemove.clear();
		this.positionsToCheck.clear();
		
		//  TODO: MASSIMO. Before moving cars, check whether there is a collision due to different velocities.
		HashMap<Car,Car> fastcars = new HashMap<Car,Car>(); // Pair of cars that due to velocity are going to collide.
		for(Car car : travelingCars)
		{
			int x = car.getX();
			int y = car.getY();
			int aux;
			Car car_front, car_right, car_left;
			String nextAction = car.getNextAction().toString();
  		
  		boolean condition=false, condition_aux = false;
  		int maxD=2; // Max distance to verify: should be max velocity -1.
  		
  		car_front = checkFront(x,y,maxD,car);
  		if (car_front!=null){
  
				condition = firstCondition(car.getSpeed(), car_front.getSpeed(), y, car_front.getY());
				if (condition){
					if (nextAction=="Go" && car_front.getNextAction().toString()=="Go"){
						
							// METODO: anyadir los vehiculos a un mapa para no mover los que van mas rapidos y luego colocarlos en la posicion nueva de los lentos.
							fastcars.put(car,car_front);
						
					}
					else if (nextAction=="Lane2Right"){
						car_right = checkFront(x+1,y,maxD,car);
						condition_aux = firstCondition(car.getSpeed(), car_right.getSpeed(), y, car_right.getY());
						if (condition_aux && car_right.getNextAction().toString()=="Go"){
							if (car_front.getNextAction().toString()=="Lane2Right"){
								aux=car_front.getSpeed()-car_right.getSpeed();
								if (aux>0){
									
										// METODO
										fastcars.put(car,car_right);
									
								}
								else {
									fastcars.put(car,car_front);
								}
							}				
						
							else {
								fastcars.put(car,car_right);
							}
						}
						
						else if (car_front.getNextAction().toString()=="Lane2Right"){
							// METODO
							fastcars.put(car,car_front);
						}
					}
					else if (nextAction=="Lane2Left"){
						car_left = checkFront(x-1,y,maxD,car);
						condition_aux = firstCondition(car.getSpeed(), car_left.getSpeed(), y, car_left.getY());
						if (condition_aux && car_left.getNextAction().toString()=="Go"){
							if (car_front.getNextAction().toString()=="Lane2Left"){
								aux=car_front.getSpeed()-car_left.getSpeed();
								if (aux>0){
									
										// METODO
										fastcars.put(car,car_left);
									
								}
								else {
									fastcars.put(car,car_front);
								}
							}				
						
							else {
								fastcars.put(car,car_left);
							}
						}
						
						else if (car_front.getNextAction().toString()=="Lane2Left"){
							// METODO
							fastcars.put(car,car_front);
						}
					}
				}
  		}
		}
			
		
		
		// Cars compute their new position
		for(Car car : travelingCars)
			//TODO: MASSIMO. Solo moverlos si no van a chocar debido a velocidad.
			if (!fastcars.containsKey(car)){
				car.move();
			}
			// car.move();
		
		for (Car car : fastcars.keySet()){
			CarPosition pos = new CarPosition(fastcars.get(car).getX(),fastcars.get(car).getY(),Direction.North);
			car.setPosition(pos); // Putting the position of the slower car to the faster one.
		}
		
		
		// Move cars in the map
		for(Car car : travelingCars)
		{
			int x = car.getX();
			int y = car.getY();

			// Move car
			if(this.isPositionOutOfBounds(x, y)) {
				relocate(car);	// Recolocamos el coche en la misma columna, abajo
			}
//			map.moveTo(car, x, y); // 
			map.moveTo(car, car.getX(), car.getY()); // **VISTO** JAVI: Modifico para que coja la nueva coordenada del coche 
			this.addPositionToCheck(car.getPosition().getGridPoint());
//			else {
//				this.addCarToRemove(car);
		}
		
		// Remove cars out of bounds
		for(Car car : this.carsToRemove)	{
			if(car.getY()>18) {
				relocate(car);
			}
			else { // TODO: MASSIMO. HAY QUE HACER REMOVE CUANDO SALGA POR LOS LATERALES!!!!!!!!!!!
				remove(car);
			}
		}

		// Manage collisions
		for(String posId : this.positionsToCheck.keySet())
		{
			GridPoint pos = this.positionsToCheck.get(posId);
			int x = pos.getX();
			int y = pos.getY();

			if(this.getNumElements(x, y) > 1)
			{
				Collision col = new Collision(x, y, map);
				Iterable<TrafficElement> elements = map.getObjectsAt(x,y);
				remove(elements);
				
				context.add(col);
				map.moveTo(col, x, y);

				this.collisions.add(col);
			}
		}
	// Manage collisions due to different velocities TODO: MASSIMO
		
	} 

	/**
	 * Emits new cars
	 */
	public void emitCars() {
		int numAddedCars = 0;
		int numAvailableCars = availableCars.size();

		// Emit cars every N steps
		if(TrafficSimulator.getTick() == (long)(this.lastCarEmitTick +
				(long)TrafficNormSynthesisSettings.SIM_NEW_CARS_FREQUENCY)) {

			int carsToEmit = Math.min(
					TrafficNormSynthesisSettings.SIM_NUM_CARS_TO_EMIT, 4);
			
			CarPosition cp = null;
			CarPosition ce = null; /*Ojo con esto, no se si un null nos sirve en ce, aunque para instanciar esta bien*/
			

			this.lastCarEmitTick = TrafficSimulator.getTick();

			while(numAvailableCars > 0 && numAddedCars < carsToEmit){
				// cp = getFreeRandomStartPoint();  // 
				cp = getFreeRandomEntryPoint(); // TODO: Massimo
				ce = getRandomExitPoint(cp); // TODO: Massimo
				numAddedCars++;

				
				
				// Cancel, since no starting points are free
				if(cp == null)
					break;  
				else {
					// add(cp);
					int des_vel = desiredVelocity();//TODO:MASSIMO
					add(cp,ce,des_vel); 
//					System.out.println("    > Car emited");
				}
				
				
				
			}		
		}
	}

	/**
	 * Adds a list of norms to all the cars in the simulation
	 * 
	 * @param norms
	 */
	public void broadcastAddNorm(Norm norm) {
		for(Car c : allCars) {
			c.getReasoner().addNorm(norm);
		}
	}

	/**
	 * Adds a list of norms to all the cars in the simulation
	 * 
	 * @param norms
	 */
	public void broadcastRemoveNorm(Norm norm) {
		for(Car c : allCars) {
			c.getReasoner().removeNorm(norm);
		}
	}

	//----------------------------------------------------------
	// Add and remove
	//----------------------------------------------------------

	/**
	 * 
	 * @param pos
	 */
	private void add(CarPosition pos)
	{
		Car car = availableCars.pop();
		car.init(pos);
		add(car);
	}

	/**
	 * TODO: MASSIMO.
	 * @param pos
	 */
	private void add(CarPosition cp, CarPosition ce, int des_vel)
	{
		Car car = availableCars.pop();
		car.init(cp, ce, des_vel);
		add(car);
	}
	
	/**
	 * Checks first condition for collision due to velocities.
	 * 
	 *
	 */
	
	private boolean firstCondition(int vel1, int vel2, int pos1, int pos2){
		boolean first=false;
		if (pos2-pos1<vel1-vel2){
			first=true;
		}
		
		return first;
	}
	
	/**
	 * Adds a car to the simulation
	 * 
	 * @param car
	 */
	private void add(Car car)
	{
		travelingCars.add(car);
		context.add(car);
		map.moveTo(car, car.getX(), car.getY());
	}
	
	/**
	 * 
	 * @param pos
	 */
	private void addPositionToCheck(GridPoint pos)
	{
		String s = pos.getX() + "-" + pos.getY();
		this.positionsToCheck.put(s, pos);
	}
	
	/**
	 * 
	 * @param elements
	 */
	private void remove(Iterable<TrafficElement> elements)
	{
		List<TrafficElement> toRemove = new ArrayList<TrafficElement>();
		
		for(TrafficElement element : elements)
			toRemove.add(element);
		
		for(TrafficElement element : toRemove)
			remove(element);			
	}
	
	/**
	 * 
	 * @param element
	 */
	private void remove(TrafficElement element)
	{
		context.remove(element);
	
			if(element instanceof Car) {
				removeCar((Car)element);
			}
	}
	
	/**
	 * MASSIMO 
	 * @param car
	 */
	private void relocate(Car car) { //TODO: Massimo.
//		CarPosition newPos = new CarPosition(car.getX(),car.getY()-18,Direction.North); //18 Because the size of the matrix is 19-1.
		CarPosition newPos = new CarPosition(car.getX(),car.getY()-19,Direction.North); // **VISTO** JAVI: Cambiado para que el coche entre en la y=0
		car.setPosition(newPos);
//		carsToRemove.remove(car); //Lo quitamos de la lista de coches por eliminar del mapa. **VISTO** JAVI: No hace falta.
	}
	
	/**
	 * Removes a car from the simulation and its position
	 * 
	 * @param car
	 */
	private void removeCar(Car car)
	{
		travelingCars.remove(car);
		availableCars.addLast(car);
	}

	/**
	 * Removes collided cars from the car map
	 */
	public void removeCollisions()
	{
		this.collisionsToRemove.clear();
		
		for(Collision col : this.collisions) {
			this.collisionsToRemove.add(col);
		}

		for(Collision col : this.collisionsToRemove) {
			this.removeCollision(col);
		}
	}
	
	/**
	 * 
	 * @param col
	 */
	private void removeCollision(Collision col)
	{
		context.remove(col);
		this.collisions.remove(col);
	}
	
	/**
	 * 
	 * @param car
	 */
	private void addCarToRemove(Car car)
	{
		this.carsToRemove.add(car);
	}

	//----------------------------------------------------------
	// Getters and setters
	//----------------------------------------------------------

	/**
	 * 
	 * @param x
	 * @param y
	 * @return
	 */
	private int getNumElements(int x, int y)
	{
		Iterable<TrafficElement> elements = map.getObjectsAt(x,y);
		int elemsCount = 0;

		Iterator<TrafficElement> iterator = elements.iterator();
		while(iterator.hasNext()) {
			iterator.next();
			elemsCount++;
		}
		return elemsCount;
	}
	
	/**
	 * 
	 * @param row
	 * @param col
	 * @return
	 */
	public TrafficElement getElement(int row, int col) {
		return this.map.getObjectAt(col, yDim-1-row);
	}
	
	/**
	 * 
	 * @param id
	 * @return
	 */
	public Car getCar(long id) {
		for(Car car : travelingCars) {
			if(car.getId() == id)
				return car;
		}
		return null;
	}

	/**
	 * Returns whether there is an agent inside the 'complete' frontal view (including laterals). TODO: MASSIMO
	 * 
	 * @return
	 */
	
	private Car checkFront(int x, int y, int maxD, Car car){
	
		Car car_aux=null;
		boolean condition;
		int aux;
		TrafficElement element;	
		for(int i=1;i<=maxD;i++){
				aux=y+i;
				if (aux>18){
					aux=aux-19;
				}
				if(this.getNumElements(x, aux) != 0){
					
					element = map.getObjectsAt(x,aux); //AQUI ASUMO QUE SOLO HAY UN VEHICULO EN LA CELDA PORQUE ES EL ESTADO DONDE YA SE HAN RESUELTO LOS CHOQUES.
					car_aux=(Car) element;
					break;
				
				}
			}
		
		
		
		// WARNING: SHORT ID HAY QUE SACARLO DE ELEMENTS!!!! MASSIMO
		return car_aux;
	}
	
		
	/**
	 * Returns the number of cars currently driving into the scenario
	 * 
	 * @return
	 */
	public int getNumCars() {
		return travelingCars.size();
	}
		
	/**
	 * Returns the start point in the map for a given direction
	 */
	/*
	private CarPosition getStartPoint(Direction dir){
		int tx = 0,ty = 0;
		switch(dir)
		{
		case North:
			tx = leftLane;
			ty = yDim-1;
			break;
		case East:
			tx = xDim-1;
			ty = upperLane;
			break;
		case South:
			tx = rightLane;
			ty = 0;
			break;
		case West: 
			tx = 0;
			ty = lowerLane;
			break;
		}
		CarPosition cp = new CarPosition(tx,ty,dir.getOppositeDirection());
		return cp;
	}
*/
	private CarPosition getStartPoint(){ //TODO: Massimo. En principio, como no pide input java no deber�a confundirlo con el otro m�todo.
		
		int pos_0x= (int) Math.round(Math.random()); //This aux variable help to obtain a random column to start.
//		pos_0x=2*pos_0x+9; //The '2' and '9' are due to the available map.
//		int pos_0y= (int) Math.round(Math.random()*19); // This aux variable help to obtain a random row to start.
		pos_0x=2*pos_0x+8; // **VISTO** JAVI: Te lo he cambiado a 8 porque el carril estaba desplazado a la derecha
		int pos_0y= (int) Math.round(Math.random()*18); // **VISTO** JAVI: Cambiado porque con el 19 un coche podia entrar fuera del mapa
							
		CarPosition cp = new CarPosition(pos_0x,pos_0y,Direction.North); //HAY QUE ASEGURARSE QUE 'X' REPRESENTA COLUMN, 'Y' FILA. Y QUE SE NAVEGA DE IZQ. A DER. Y DE ARRIBA A ABAJO.
		
		return cp;
	}
	
	/**
	 * Returns a free random start point in the map
	 * 
	 * @return
	 */
	/*
	private CarPosition getFreeRandomStartPoint()
	{
		Direction dir = Utilities.getRandomDirection();
		CarPosition cp = getStartPoint(dir);
		int cnt = 0;

		while(!isFree(cp.getGridPoint()) && cnt++ < 3) {
			cp = getStartPoint(dir = Utilities.getTurnDirection(dir, Turn.Right));
		}
		if(!isFree(cp.getGridPoint()))
			cp = null;

		return cp;
	}
	*/

	/**
	 * Este metodo tienes que llamarlo en el emitCars() en vez del getFreeRandomStartPoint
	 * @return
	 */
	private CarPosition getFreeRandomEntryPoint() { 
		/* **VISTO** JAVI: lo que tenias del bucle 		"while(!isFree(cp.getGridPoint()) && cnt++ < 3) {"
		 * hacia que, si el punto de entrada devuelto no estaba libre, intentaba buscar otro 
		 * alternativo hasta 4 veces. Esto era para el escenario antiguo, en que si un coche no
		 * podia entrar por un punto, se intentaba hacerlo entrar por los tres restantes
		 */
		CarPosition cp = getStartPoint(); // Obtener punto de entrada
		if(!isFree(cp.getGridPoint())) {	// Si no esta libre ponerlo a null
			cp = null;
		}
		return cp;
	}
	
	/** TODO: Massimo
	 * This method returns a random 'desired' velocity
	 * @return
	 */
	private int desiredVelocity(){
		int max_vel=3; // Max. number of cells a car can move each tick.
		int des_vel = (int) (Math.round(Math.random()*max_vel));
		
		return des_vel;
		
	}
	
	/**
	 * This method returns a random point to exit the system.
	 * @return
	 */
	private CarPosition getRandomExitPoint(CarPosition cp) { 
		CarPosition ce = null;
		
		if(cp!=null) {
			int dist = (int) (Math.round(Math.random()*80)); //Chooses a random distance (the max equals to 5 times the length.
			dist=20+dist; //For creating the RND point within a range.
			int lane= (int) Math.round(Math.random()); //This aux variable help to obtain a random column to exit.
			lane=2*lane+9; //The '2' and '9' are due to the available map.
			
			ce = new CarPosition(lane,dist+cp.getY(),Direction.North);
			/**
			 * Hay que ver como afecta lo del null, porque a lo mejor no me te el veh�culo y deber�a devolver un ce=null tambi�n.
			 * 
			 * **VISTO** JAVI: Si cp es null no entrara dentro de este if, por lo que acabara devolviendo null mas abajo en "return ce" :)
			 */
		}	
		return ce;
	}
	
	/**
	 * Returns (x,y) values for initial start point
	 * 
	 * @param p
	 * @return
	 */
	
	
	/**
	 * Returns true if the car is out of the map
	 * 
	 * @param p
	 * @return
	 */
	public boolean isPositionOutOfBounds(int x, int y)
	{
		if(x<0 || y < 0 || x >= xDim || y >= yDim)
			return true;
		return false;
	}
	
	/**
	 * Returns true if a grid point of the map is free (with no car) 
	 * 
	 * @param p
	 * @return
	 */
	private boolean isFree(GridPoint p)
	{
		int count = 0;
		Iterable<TrafficElement> elements = map.getObjectsAt(p.getX(),p.getY());
		for(TrafficElement elem : elements) {
			count++;
		}
		return count==0;
	}

	/**
	 * Returns the x dimension of the map
	 * 
	 * @return  the xDim
	 */
	public int getXDim() 
	{
		return xDim;
	}

	/**
	 * Returns the y dimension of the map
	 * 
	 * @return  the yDim
	 */
	public int getYDim() 
	{
		return yDim;
	}

	/**
	 * Returns the lower lane of the map
	 * 
	 * @return  the lowerLane
	 */
	public int getLowerLane() 
	{
		return lowerLane;
	}

	/**
	 * Returns the upper lane of the map
	 * 
	 * @return  the upperLane
	 */
	public int getUpperLane() 
	{
		return upperLane;
	}

	/**
	 * Returns the left lane of the map
	 * 
	 * @return  the leftLane
	 */
	public int getLeftLane() 
	{
		return leftLane;
	}

	/**
	 * Returns the right lane of the map
	 * 
	 * @return  the rightLane
	 */
	public int getRightLane()
	{
		return rightLane;
	}
}
