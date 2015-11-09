package es.csic.iiia.normlab.traffic.agent;

import java.awt.Color;
import java.util.Random;

import repast.simphony.annotate.AgentAnnot;
import repast.simphony.space.grid.GridPoint;
import es.csic.iiia.normlab.traffic.TrafficSimulator;
import es.csic.iiia.normlab.traffic.car.CarAction;
import es.csic.iiia.normlab.traffic.car.CarColor;
import es.csic.iiia.normlab.traffic.car.CarPosition;
import es.csic.iiia.normlab.traffic.car.CarReasoner;
import es.csic.iiia.normlab.traffic.car.CarReasonerState;
import es.csic.iiia.normlab.traffic.car.context.CarContext;
import es.csic.iiia.normlab.traffic.factory.CarContextFactory;
import es.csic.iiia.normlab.traffic.factory.TrafficFactFactory;
import es.csic.iiia.normlab.traffic.map.CarMap;
import es.csic.iiia.normlab.traffic.utils.Direction;
import es.csic.iiia.normlab.traffic.utils.Speed;
import es.csic.iiia.normlab.traffic.utils.Turn;
import es.csic.iiia.normlab.traffic.utils.Utilities;
import es.csic.iiia.nsm.agent.EnvironmentAgent;
import es.csic.iiia.nsm.agent.language.PredicatesDomains;
import es.csic.iiia.nsm.norm.Norm;

/**
 * The car agent
 *
 * @author Javier Morales (jmoralesmat@gmail.com)
 *
 */
@AgentAnnot(displayName = "Car Agent")
public class Car implements TrafficElement, EnvironmentAgent
{
	//-----------------------------------------------------------------
	// Attributes
	//-----------------------------------------------------------------
	
	protected PredicatesDomains predDomains;
	protected TrafficFactFactory factFactory;
	protected CarContextFactory contextFactory;
	
	protected CarReasoner reasoner;
	protected CarPosition position;
	protected CarPosition exit; //TODO: Massimo
	protected CarContext context;
	protected CarColor color;

	protected CarReasonerState reasonerState;
	protected Turn turn;

	//	private Action lastPerformedAction;
	protected CarAction nextAction;

	protected boolean hasTurned;
	protected boolean collided; 

	protected Norm normToViolate;
	protected Norm normToApply;
	
	protected short id;
	protected int defaultSpeed;
	protected int speed;
	protected int desiredVel;
	protected int distance; // TODO: MASSIMO. La idea es que cada vehiculo sepa cuanto ha recorrido.
	protected boolean[] lane; // TODO: MASSIMO. Este seria un vector de 3 posiciones donde un '1' indicara que el carril es valido.
	
	
	protected int ticksCollided;
	protected int expectedArrivalTicks;
	protected int numTicksTraveling;

	protected CarPosition exitPoint; // Punto de salida del coche, Puesto aleatoriamente en init()
	
	//-----------------------------------------------------------------
	// Constructors
	//-----------------------------------------------------------------
	
	/**
	 * This constructor
	 */
	public Car(short id, boolean withReasoner, PredicatesDomains predDomains,
			CarContextFactory contextFactory, TrafficFactFactory factFactory) {
		
		this.contextFactory = contextFactory;
		this.factFactory = factFactory;
		this.predDomains = predDomains;
		this.id = id;
		this.distance=0; // TODO: MASSIMO. La idea es que cada vehiculo sepa cuanto ha recorrido.
		//this.lane=this.method();
		
		this.hasTurned = false;
		this.collided = false;
		this.ticksCollided = 0;
		this.defaultSpeed = 1;
		this.numTicksTraveling = 0;
		this.speed = this.defaultSpeed;
		this.color = new CarColor();
	
		
		this.reasoner = new CarReasoner(predDomains, factFactory);
		this.reasonerState = CarReasonerState.NoNormActivated;
		this.nextAction = CarAction.Go;

		// Reasoner for the car. Only create it if it's not a clone. If
		// it's a clone, it doesn't need the reasoner, so due to details of
		// efficiency we don't assign anyone to it
		//		if(withReasoner) {
		//			this.reasoner = new CarReasoner(this);
		//		}
	}


	//-----------------------------------------------------------------
	// Scheduled methods
	//-----------------------------------------------------------------

	public void move()
	{
		CarMap carMap = TrafficSimulator.getMap();

		this.incTicks(); 

		// Execute last decided action and turn if needed
		this.execute(nextAction);
		if (nextAction.toString()!="stop"){
			this.distance=this.distance+this.speed;/* TODO: MASSIMO. La idea es que cada vehiculo sepa cuanto ha recorrido. OJO LA IDEA
			ES QUE CUANDO SE MUEVA LATERALMENTE LO HAGA HACIA DELANTE EN DIAGONAL, SI NO ESTE IF NO SERIA CORRECTO*/
		}
		
		this.turn(carMap);
		
	}
	
	/**
	 * Reason about the current situation. Then decide what to do
	 */
	public void perceiveAndReason() {
		CarMap carMap = TrafficSimulator.getMap();

		// Perceive and reason to get the action to apply in the next step
		this.context = this.perceive(carMap);
		this.nextAction = this.reason();
	}

	/**
	 * 
	 * @param action
	 */
	private void execute(CarAction action) { //TODO: Massimo.

		// Apply the next action to do
		switch (this.nextAction)
		{
		case Go:
			//this.speed = defaultSpeed;
			movement();
			break;

		case Stop:
			this.speed = reasonVel(this.nextAction,this.context);
			break;
			
		case Lane2Left:
			//this.speed= defaultSpeed;
			movement(false); // 'False' Representa cambiar al carril izq.
			break;
			
		case Lane2Right:
			//this.speed= defaultSpeed;
			movement(true); // 'True' Representa cambiar al carril der.
			break;
			
		case Decelerate:
			this.speed = reasonVel(this.nextAction,this.context);
			break;
			
		case Accelerate:
			this.speed = reasonVel(this.nextAction,this.context);
			break;
		
		default:
			break;
		}
	}

	/**
	 * The car perceives the environment 
	 */
	private CarContext perceive(CarMap carMap) {
		return this.generateScope(carMap);
	}

	/**
	 * Makes the car reason about its current situation
	 */
	private CarAction reason() {
		CarAction action;

		// Decide next action with the reasoner
		//action = reasoner.decideAction(this, context);
		
		action = reasoner.decideAction(this, context,lane);
		this.reasonerState = reasoner.getState();

		this.normToViolate = reasoner.getNormToViolate();
		this.normToApply = reasoner.getNormToApply();
		
		return action;
	}
	
	/**
	 * Makes the car reason about its current situation too, focusing on the velocities issue
	 */
	
	private int reasonVel(CarAction action,CarContext context){
		
		// CODIGO DE IF PARA GENERAR LA VELOCIDAD A PARTIR DE LA ACCION. SI STOP VEL = 0, SI DECELERATE MIRAR CONTEXTO
		// Y COGER LA VEL DEL DE DELANTE, AND SO ON.
		
		
		
		return new_vel;
	}
	
	//-----------------------------------------------------------------
	// Methods
	//-----------------------------------------------------------------

	/**
	 * Initializes the car //TODO: Massimo.
	 */
	public void init(CarPosition cp, CarPosition ce, int des_vel) {
		this.position = cp;
		this.exit=ce;
		this.collided = false;
		desiredVel=des_vel; // Velocity that the agent wants to maintain on its travel.
		this.speed=desiredVel; // The initial speed is equal to the desired velocity.
		hasTurned=true; /*OJO!! NO SE SI FUNCIONA --> 
		Pequenya trampa para que no salte el metodo turn() y no verifique el 'turn' que aqui no se asigna.*/
	}
	
	/**
	 * Initializes the car 
	 */
	public void init(CarPosition position) {
		this.position = position;
		this.collided = false;

		this.turn = Utilities.getRandomTurn();
		
		// Set expected arrival time
		switch(this.turn) {
		case Left:
			this.expectedArrivalTicks = 23;
			break;
		case Straight:
			this.expectedArrivalTicks = 21;
			break;
		case Right:
			this.expectedArrivalTicks = 19;
			break;
		}
	}

	/**
	 * Moves the car one step in the direction that it's oriented
	 */
	private void movement() {
		position = position.add(getMovementVector());
	}
	private void movement(boolean side) { //TODO: MASSIMO. Para cuando le pida como accion un cambio de carril.
		if (side=false){
			position = position.add(getMovementVector(side)); //Se mueve arriba a la izq.
		}
		else{
			position = position.add(getMovementVector(side)); //Se mueve arriba a la der.
		}
			
	}

	/**
	 * Check if has to turn and turn if so. Turns the car to the direction it has to turn
	 */
	public void turn(CarMap carMap) {
		int x = position.getX();
		int y = position.getY();

		if(!hasTurned) {
			if( (position.getDirection()==Direction.North && ((turn==Turn.Right && y == carMap.getLowerLane()) ||
					((turn==Turn.Left || turn==Turn.Straight) && y==carMap.getUpperLane()))) ||
					(position.getDirection()==Direction.South && ((turn==Turn.Right && y == carMap.getUpperLane()) ||
							((turn==Turn.Left || turn==Turn.Straight) && y==carMap.getLowerLane()))) ||
							(position.getDirection()==Direction.East && ((turn==Turn.Right && x == carMap.getLeftLane()) ||
									((turn==Turn.Left || turn==Turn.Straight) && x==carMap.getRightLane()))) ||
									(position.getDirection()==Direction.West && ((turn==Turn.Right && x == carMap.getRightLane()) ||
											((turn==Turn.Left || turn==Turn.Straight) && x==carMap.getLeftLane()))))
			{
				position.turn(turn);
				hasTurned=true;
			}
		}
	}

	/**
	 * Calculates the movement vector of the car
	 */
	private GridPoint calcMovementVector() {
		int movement[] = Utilities.getDirVector(position.getDirection());
		for(int i=0; i<movement.length; i++)
			movement[i] *= speed;
		return new GridPoint(movement[0],movement[1]);
	}
	
	/**
	 * Calculates the movement vector of the car taking into account the lateral displacement// TODO: MASSIMO
	 */
	private GridPoint calcMovementVector(boolean side) {
		int movement[] = Utilities.getDirVector(position.getDirection());
		byte lateral;
		if (side=false){
			lateral=-1;
		}
		else{
			lateral=1;
		}
		for(int i=0; i<movement.length; i++)
			movement[i] *= speed;
		
		return new GridPoint(movement[0]+lateral,movement[1]);
	}

	/**
	 * Generates the car scope in the current moment
	 */
	public CarContext generateScope(CarMap carMap) {
		CarContext context = contextFactory.getCarContextIn(carMap, id, CarContext.Type.Front);
		return context;
	}

	/**
	 * 
	 */
	private void incTicks() {
		this.numTicksTraveling++;
	}

	//-----------------------------------------------------------------
	// Getters and setters
	//-----------------------------------------------------------------

	/**
	 * Returns the available lanes
	 * TODO: MASSIMO. La idea es que diga que carriles puede usar el vehiculo en cada tick.
	 * 
	 * @return
	 */
	private void availableLanes(){
		// OJO!! ES POSIBLE QUE PODAMOS FIJAR EL LANE MAS ARRIBA Y ESTO NO HAGA FALTA.
		boolean [] aux_lane = new boolean [3];
		if(this.numTicksTraveling==0){ //Aqui ponemos como disponible nada mas introducir el vehiculo, el carril donde entra.
			if(this.getX()==8){ //8 because of the map.
				this.lane=new boolean[] {true,false,false};
			}
			else{
				this.lane=new boolean[] {false,false,true};
			}
		}
		else{ //Modificaciones semi-random de carriles validos.
			for(int i=0; i<3; i++){
				aux_lane[i]=changeLaneProb(i); // Generamos la prob de que un carril sea valido.
			}
			if (aux_lane[0]==true && aux_lane[1]==false && aux_lane[2]==true){ // En caso de que se de esta combinacion por la aleatoriedad, se corrige. Es una situacion no realista.
				aux_lane[1]=true;
			}
			else if (aux_lane[1]==false && ((aux_lane[0]==true && this.lane[2]==false) || (this.lane[0]==true && aux_lane[2]==false))){
				aux_lane[1]=true;
				
			}
			else if (aux_lane[0]==false && aux_lane[1]==false && aux_lane[2]==false){
				while (aux_lane[0]==false && aux_lane[1]==false && aux_lane[2]==false){
					for(int i=0; i<3; i++){
						aux_lane[i]=changeLaneProb(i); // Generamos la prob de que un carril sea valido.
					}
				}
				
			}
			this.lane=aux_lane;
		}
		
		
		
	}
	
	private boolean changeLaneProb(int x){ //TODO: MASSIMO. 
		double prob;
		boolean available;
		double aux = (this.exit.getY()-this.distance)/this.exit.getY(); // Sirve para medir si ha recorrido la distancia m�nima a recorrer.
		if (aux>=0.3){ //Si esta mas lejos de un 30% del recorrido minimo, los carriles validos son random.
			prob = Math.random();
		}
		else if (aux>0){ //Si esta entre un 0 y un 30%, los carriles validos son el central y el pertinente.
			if(x==this.exit.getX()-8||x==1){ // -8 por la definicion del mapa.
				prob=1;
			}
			else{
				prob=0;
			}
		}
		else{ // Si ya llego a la altura o la ha sobrepasado, el unico carril valido es el pertinente.
			if(x==this.exit.getX()-8){
				prob=1;
			}
			else{
				prob=0;
			}
		}
		
		double aux2=Math.random(); // Tira la moneda y comprueba si un carril es valido o no.
		if (aux2<prob){
			available=true;
		}
		else{
			available=false;
		}
		return available;
	}
	/**
	 * Returns the movement vector of the car
	 * 
	 * @return
	 */
	private GridPoint getMovementVector(){
		return calcMovementVector();
	}
	
	private GridPoint getMovementVector(boolean side){ //TODO: MASSIMO
		return calcMovementVector(side);
	}

	/**
	 * Returns the estimated speed of the car
	 * 
	 * @return
	 */
	public Speed getEstimatedSpeed() {
		if(speed == 0)
			return Speed.None;
		else if(speed == 1)
			return Speed.Medium;
		else if(speed == 2)
			return Speed.High;
		else
			return Speed.VeryHigh;
	}

	/**
	 * Returns the estimated speed of the car relative to the other car's speed
	 * 
	 * @return
	 */
	public Speed getEstimatedSpeed(int otherSpeed) {
		if(speed == 0)
			return Speed.None;

		int diff = speed - otherSpeed;
		switch(diff) {
		case -2:	return Speed.MuchLower;
		case -1:	return Speed.Lower;
		case 0:		return Speed.Equal;
		case 1:		return Speed.Higher;
		case 2:		return Speed.MuchHigher;
		default:	return Speed.None;
		}
	}

	/**
	 * Returns the car scope
	 */
	public CarContext getFrontScope() {
		return context;
	}

	/**
	 * 
	 * @return
	 */
	public int getSpeed() {
		return speed;
	}

	/**
	 * Returns the position of the car at the moment
	 * 
	 * @return
	 */
	public CarPosition getPosition() {
		return position;
	}

	/**
	 * 
	 * @return
	 */
	public CarReasoner getReasoner() {
		return reasoner;
	}

	/**
	 * 
	 * @return
	 */
	public long getId() {
		return id;
	}

	/**
	 * Returns the x coordinate of the car
	 */
	public int getX() {
		return position.getX();
	}

	/**
	 * Returns the y coordinate of the car
	 */
	public int getY() {
		return position.getY();
	}

	/**
	 * Returns a string describing the object
	 */
	@Override
	public String toString(){
		return this.position.getDirection().getArrow();
	}

	/**
	 * Returns a extended string describing the object
	 * @return
	 */
	public String toStringExt(){
		return "Pos: " + position + " turn: " + turn.name() + " has turned: " + hasTurned;		
	}

	/**
	 * Returns true if the car has collided
	 */
	public boolean isCollided() {
		return collided;
	}

	/**
	 * @param collided the collided to set
	 */
	public void setCollided(boolean collided) {
		this.collided = collided;
		if(collided) {
			this.ticksCollided = 0;
		}
	}

	/**
	 * 
	 * @return
	 */
	public int getTicksCollided() {
		return this.ticksCollided;
	}

	/**
	 * 
	 * @return
	 */
	public void setTicksCollided(int ticksCollided) {
		this.ticksCollided = ticksCollided;
	}

	/**
	 * 
	 */
	public void setSpeed(int speed) {
		this.speed = speed; 
	}

	/**
	 * @param pos
	 */
	public void setPosition(CarPosition pos){
		position = pos;
	}

	/**
	 * 
	 * @return
	 */
	public CarAction getNextAction() {
		return this.nextAction;
	}

	/**
	 * 
	 * @return
	 */
	public CarReasonerState getReasonerState() {
		return this.reasonerState;
	}

	/**
	 * 
	 * @param state
	 */
	public void setReasonerState(CarReasonerState state) {
		this.reasonerState = state;
	}

	/**
	 * 
	 * @param color
	 */
	public void setCarColor(CarColor color) {
		this.color = color;
	}

	/**
	 * 
	 * @return
	 */
	public Color getColor() {
		return this.color.getColor();
	}

	/**
	 * 
	 * @return
	 */
	public int getExpectedArrivalTicks() {
		return this.expectedArrivalTicks;
	}

	/**
	 * 
	 * @return
	 */
	public int getFinalArrivalTicks() {
		return this.numTicksTraveling;
	}

	/**
	 * 
	 * @return
	 */
	public boolean isCasualStop() {
		return this.reasoner.isCasualStop();
	}

	/**
	 * 
	 * @return
	 */
	public int getNumTicksTraveling() {
		return this.numTicksTraveling;
	}


	/**
	 * Returns the last norm that was violated by the car
	 * 
	 * @return
	 */
	public Norm getNormToViolate() {
		return this.normToViolate;
	}

	/**
	 * Sets the last violated norm
	 * 
	 * @param n
	 */
	public void setNormToViolate(Norm n) {
		this.normToViolate = n;
	}

	/**
	 * Returns the last norm that was applied by the car
	 * 
	 * @return
	 */
	public Norm getNormToApply() {
		return this.normToApply;
	}

	/**
	 * Sets the last applied norm
	 * 
	 * @param n
	 */
	public void setNormToApply(Norm n) {
		this.normToApply = n;
	}
	/**
	 * Clones the car
	 */
	public Car clone(boolean withScopes) {
		CarPosition pos = new CarPosition(position.getX(), position.getY(),
				position.getDirection());

		Car car = new Car(this.id, false, predDomains, contextFactory,
				factFactory);
		car.init(pos);

		car.setReasonerState(this.reasonerState);
		car.setSpeed(this.speed);
		car.setCollided(this.collided);
		car.setTicksCollided(this.ticksCollided);
		car.setCarColor(this.color);

		if(withScopes && this.getFrontScope() != null) {
			car.context = context;
		}
		return car;
	}
}
