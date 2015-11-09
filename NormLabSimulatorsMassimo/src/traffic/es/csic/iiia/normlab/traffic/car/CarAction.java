package es.csic.iiia.normlab.traffic.car;

import es.csic.iiia.nsm.agent.EnvironmentAgentAction;

/**
 * Defines an action to be taken by an agent
 * 
 * @author Javier Morales (jmoralesmat@gmail.com)
 *
 */
public enum CarAction implements EnvironmentAgentAction { //TODO: Massimo. He anyadido Lane2Left y Lane2Right para los cambios.

	Nothing, Go, Stop, Accelerate, Decelerate, TurnLeft, TurnRight, Lane2Left,Lane2Right;

	/**
	 * Returns the opposite action to the passed action
	 * 
	 * @param action
	 * @return
	 */
	public CarAction getOpposite() {

		switch(this) {
		case Stop:				return Go;
		case Go:					return Stop;
		case Accelerate:	return Decelerate;
		case Decelerate: 	return Accelerate;
		case TurnLeft:		return TurnRight;
		case TurnRight:		return TurnLeft;
		case Lane2Left: 	return Lane2Right;
		case Lane2Right: 	return Lane2Left;
		default:					return Nothing;
		}
	}
	
	/**
	 * To string method
	 */
	public String toString() {
		switch(this) {
		case Go: 					return "Go";
		case Stop: 				return "Stop";
		case Accelerate:	return "Accelerate";
		case Decelerate:	return "Decelerate";
		case TurnLeft: 		return "TurnLeft";
		case TurnRight:		return "TurnRight";
		case Lane2Left: 	return "Lane2Left";
		case Lane2Right: 	return "Lane2Right";
		default:					return "Nothing";
		}
	}

	/**
	 * To string method
	 */
	public String toStringExt() {
		switch(this) {
		case Go: 					return "Go";
		case Stop: 				return "Stop";
		case Accelerate:	return "Accelerate";
		case Decelerate:	return "Decelerate";
		case TurnLeft: 		return "TurnLeft";
		case TurnRight:		return "TurnRight";
		case Lane2Left: 	return "Lane2Left";
		case Lane2Right: 	return "Lane2Right";
		default:					return "Nothing";
		
		}
	}
}
