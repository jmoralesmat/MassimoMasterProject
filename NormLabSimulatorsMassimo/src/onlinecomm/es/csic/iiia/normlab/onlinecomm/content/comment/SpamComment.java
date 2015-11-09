package es.csic.iiia.normlab.onlinecomm.content.comment;


/**
 * Spammer comment
 * 
 * @author davidsanchezpinsach
 *
 * Modified by Iosu Mendizabal
 * 
 */
public class SpamComment extends Comment {
	
	public SpamComment(int id, int contentType, String file, String url, 
			String message, int section, String contentTypeDesc, double timeStep){
		super( id, contentType, file, url, message,
				section, contentTypeDesc, timeStep);
	}
}
