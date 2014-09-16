package eu.sathra.mavlink.generator;

import java.io.File;
import java.io.FileWriter;
import java.util.HashSet;
import java.util.Set;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.apache.velocity.Template;
import org.apache.velocity.VelocityContext;
import org.apache.velocity.app.VelocityEngine;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class MCG {

	private static final String ELEMENT_ROOT = "mavlink";
	private static final String ELEMENT_VERSION = "version";
	private static final String ELEMENT_ENUM = "enum";
	private static final String ELEMENT_MESSAGE = "message";
	private static final String ELEMENT_DESCRIPTION = "description";
	private static final String ATTR_NAME = "name";
	private static final String ATTR_TYPE = "type";
	private static final String ATTR_DESCRIPTION = "description";
	private static final String ATTR_FIELD = "field";
	private static final String ATTR_ID = "id";

	public static void main(String[] args) {

		try {
			File fXmlFile = new File(args[0]);
			DocumentBuilderFactory dbFactory = DocumentBuilderFactory
					.newInstance();
			DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
			Document myDocument = dBuilder.parse(fXmlFile);

			/*
			 * Initialize velocity
			 */
			VelocityEngine velocity = new VelocityEngine();
			velocity.init();
			VelocityContext velocityContext = new VelocityContext();

			/*
			 * Get protocol version
			 */
			Node versionNode = myDocument.getElementsByTagName(ELEMENT_VERSION)
					.item(0);
			velocityContext.put("version", versionNode.getTextContent());

			/*
			 * Parse enums
			 */
			NodeList docEnums = myDocument.getElementsByTagName(ELEMENT_ENUM);
			Set<MavEnum> enumsSet = new HashSet<MavEnum>();

			for (int c = 0; c < docEnums.getLength(); ++c) {
				Element myElement = (Element) docEnums.item(c);

				MavEnum myEnum = new MavEnum();
				myEnum.setName(myElement.getAttribute(ATTR_NAME));

				Node descriptionNode = myElement.getElementsByTagName(
						ATTR_DESCRIPTION).item(0);

				myEnum.setDescription(descriptionNode == null ? null
						: descriptionNode.getTextContent());

				/*
				 * Parse enum values
				 */
				NodeList docValues = myElement.getElementsByTagName("entry");

				for (int d = 0; d < docValues.getLength(); ++d) {
					Element valueElement = (Element) docValues.item(d);
					MavEnum.Value myValue = new MavEnum.Value();
					myValue.setName(valueElement.getAttribute(ATTR_NAME));
					String value = valueElement.getAttribute("value");

					try {
						myValue.setValue(Integer.parseInt(value));
					} catch (NumberFormatException e) {
						myValue.setValue(d);
					}

					Node valueDescriptionNode = valueElement
							.getElementsByTagName(ELEMENT_DESCRIPTION).item(0);

					myValue.setDescription(valueDescriptionNode == null ? null
							: valueDescriptionNode.getTextContent());
					myEnum.addValue(myValue);
				}

				enumsSet.add(myEnum);
			}

			velocityContext.put("enums", enumsSet);

			/*
			 * Parse messages
			 */

			NodeList messages = myDocument
					.getElementsByTagName(ELEMENT_MESSAGE);
			Set<MavMessage> mMavMessages = new HashSet<MavMessage>();
			Template messageTemplate = velocity
					.getTemplate("templates/message.vtl");

			Set<MavMessage> mMessages = new HashSet<MavMessage>();

			for (int c = 0; c < messages.getLength(); ++c) {
				Element myElement = (Element) messages.item(c);

				MavMessage myMavMessage = new MavMessage();
				myMavMessage.setId(Integer.parseInt(myElement
						.getAttribute(ATTR_ID)));
				myMavMessage.setName(myElement.getAttribute(ATTR_NAME));

				NodeList children = myElement.getChildNodes();

				for (int d = 0; d < children.getLength(); ++d) {
					Node childNode = children.item(d);

					switch (childNode.getNodeName()) {
					case ATTR_DESCRIPTION:
						myMavMessage.setDescription(childNode.getTextContent());
						break;

					case ATTR_FIELD:
						MavField myField = new MavField();
						myField.setName(((Element) childNode)
								.getAttribute(ATTR_NAME));
						myField.setType(((Element) childNode)
								.getAttribute(ATTR_TYPE));
						myField.setDescription(childNode.getTextContent());
						myMavMessage.addField(myField);
						break;
					}
				}

				myMavMessage.reorderFields();
				mMessages.add(myMavMessage);

				/*
				 * Calculate CRC_EXTRA 
				 */
				StringBuilder msgSignature = new StringBuilder(myMavMessage.getName() + " ");
				
				for(MavField myField : myMavMessage.getFields()) {
					msgSignature.append(myField.getType().replaceAll("_mavlink_version", "").replaceAll("\\[\\d+\\]", "") + " ");
					msgSignature.append(myField.getName() + " ");
					
					if(myField.isArray()) { msgSignature.append((char)myField.getLength()); }
				}
				
				int crc = getCRC(msgSignature.toString().getBytes());
				myMavMessage.setCRCExtra(crc);
			}

			velocityContext.put("message_list", mMessages);

			File myFile = new File(
					"generated/eu/sathra/mavlink/MavLink.java");
			myFile.getParentFile().mkdirs();
			myFile.createNewFile();

			FileWriter writer = new FileWriter(myFile, false);
			messageTemplate.merge(velocityContext, writer);
			writer.close();

			velocityContext.put("message_list", mMavMessages);

		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	public static int getCRC(byte[] bytes) {
		int crc = 0xffff;
		
		for(byte b : bytes) {
			int tmp;
			int data = b & 0xff;
			tmp = data ^ (crc & 0xff);
			tmp ^= (tmp << 4) & 0xff;
			crc = ((crc >> 8) & 0xff) ^ (tmp << 8) ^ (tmp << 3)
					^ ((tmp >> 4) & 0xf);
		}
		
		return (crc&0xFF) ^ (crc>>8) ;
	}

}
