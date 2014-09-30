package eu.sathra.mavlink.generator;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashSet;
import java.util.Set;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.apache.velocity.Template;
import org.apache.velocity.VelocityContext;
import org.apache.velocity.app.VelocityEngine;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

public class MCG {

	private static final String JAVA_TEMPLATE_PATH = "templates/java.vtl";
	private static final String OUTPUT_PATH = "generated/eu/sathra/mavlink/MAVLink.java";

	private static final String ELEMENT_VERSION = "version";
	private static final String ELEMENT_ENUM = "enum";
	private static final String ELEMENT_MESSAGE = "message";
	private static final String ELEMENT_DESCRIPTION = "description";
	private static final String TAG_ENTRY = "entry";
	private static final String ATTR_NAME = "name";
	private static final String ATTR_VALUE = "value";
	private static final String ATTR_TYPE = "type";
	private static final String ATTR_DESCRIPTION = "description";
	private static final String ATTR_FIELD = "field";
	private static final String ATTR_ID = "id";

	private static final String CONTEXT_VERSION = "version";
	private static final String CONTEXT_ENUMS = "enums";
	private static final String CONTEXT_MESSAGES_LIST = "message_list";

	public static void main(String[] args) {

		try {
			Document myDocument = loadMavlinkXML(args[0]);

			/*
			 * Initialize velocity
			 */
			VelocityEngine velocity = new VelocityEngine();
			velocity.init();
			VelocityContext context = new VelocityContext();

			/*
			 * Parse the document
			 */
			parseProtocolVersion(myDocument, context);
			parseEnums(myDocument, context);
			parseMessages(myDocument, context);
			
			/*
			 * Write the output
			 */
			System.out.println("Writting output to: " + OUTPUT_PATH);
			
			Template messageTemplate = velocity.getTemplate(JAVA_TEMPLATE_PATH);
			writeOutput(OUTPUT_PATH, messageTemplate, context);

			System.out.println("Done.");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static void parseProtocolVersion(Document document,
			VelocityContext context) {
		Node versionNode = document.getElementsByTagName(ELEMENT_VERSION).item(
				0);
		context.put(CONTEXT_VERSION, versionNode.getTextContent());
	}
	
	private static void parseEnums(Document document,
			VelocityContext context) {
		NodeList docEnums = document.getElementsByTagName(ELEMENT_ENUM);
		Set<MavEnum> enumsSet = new HashSet<MavEnum>();

		for (int c = 0; c < docEnums.getLength(); ++c) {
			Element myElement = (Element) docEnums.item(c);

			MavEnum myEnum = new MavEnum();
			myEnum.setName(myElement.getAttribute(ATTR_NAME));

			System.out.println("Parsing enum: " + myEnum.getName());
			
			Node descriptionNode = myElement.getElementsByTagName(
					ATTR_DESCRIPTION).item(0);

			myEnum.setDescription(descriptionNode == null ? null
					: descriptionNode.getTextContent());

			/*
			 * Parse enum values
			 */
			NodeList docValues = myElement.getElementsByTagName(TAG_ENTRY);

			for (int d = 0; d < docValues.getLength(); ++d) {
				Element valueElement = (Element) docValues.item(d);
				MavEnum.Value myValue = new MavEnum.Value();
				myValue.setName(valueElement.getAttribute(ATTR_NAME));
				String value = valueElement.getAttribute(ATTR_VALUE);

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

		context.put(CONTEXT_ENUMS, enumsSet);
	}
	
	private static void parseMessages(Document document, VelocityContext context) {

		NodeList messages = document
				.getElementsByTagName(ELEMENT_MESSAGE);

		Set<MavMessage> mMessages = new HashSet<MavMessage>();

		for (int c = 0; c < messages.getLength(); ++c) {
			Element myElement = (Element) messages.item(c);

			MavMessage myMavMessage = new MavMessage();
			myMavMessage.setId(Integer.parseInt(myElement
					.getAttribute(ATTR_ID)));
			myMavMessage.setName(myElement.getAttribute(ATTR_NAME));

			System.out.println("Parsing message: " + myMavMessage.getName());
			
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
			StringBuilder msgSignature = new StringBuilder(
					myMavMessage.getName() + " ");

			for (MavField myField : myMavMessage.getFields()) {
				msgSignature.append(myField.getType()
						.replaceAll("_mavlink_version", "")
						.replaceAll("\\[\\d+\\]", "")
						+ " ");
				msgSignature.append(myField.getName() + " ");

				if (myField.isArray()) {
					msgSignature.append((char) myField.getLength());
				}
			}

			int crc = getCRC(msgSignature.toString().getBytes());
			myMavMessage.setCRCExtra(crc);
		}

		context.put(CONTEXT_MESSAGES_LIST, mMessages);
	}

	private static Document loadMavlinkXML(String path)
			throws ParserConfigurationException, SAXException, IOException {
		File mavlinkXMLFile = new File(path);
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
		return dBuilder.parse(mavlinkXMLFile);
	}

	private static void writeOutput(String outputPath, Template template,
			VelocityContext context) throws IOException {
		File myFile = new File(OUTPUT_PATH);
		myFile.getParentFile().mkdirs();
		myFile.createNewFile();

		FileWriter writer = new FileWriter(myFile, false);
		template.merge(context, writer);
		writer.close();
	}

	private static int getCRC(byte[] bytes) {
		int crc = 0xffff;

		for (byte b : bytes) {
			int tmp;
			int data = b & 0xff;
			tmp = data ^ (crc & 0xff);
			tmp ^= (tmp << 4) & 0xff;
			crc = ((crc >> 8) & 0xff) ^ (tmp << 8) ^ (tmp << 3)
					^ ((tmp >> 4) & 0xf);
		}

		return (crc & 0xFF) ^ (crc >> 8);
	}

}
