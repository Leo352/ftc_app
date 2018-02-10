package org.firstinspires.ftc.team7234;

import java.io.IOException;
import java.text.Normalizer;
import java.util.logging.ConsoleHandler;
import java.util.logging.FileHandler;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

public class BotmanLogger {
    static private FileHandler fileTxt;
    static private SimpleFormatter formatterTxt;

    static private FileHandler fileHTML;
    static private Formatter formatterHTML;

    static public void setup() throws IOException {

        Logger logger = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);

        //suppress the logging output to the console
        Logger rootLogger = Logger.getLogger("");  //TODO: Check if this is right, origninal had asterisk arount getLogger
        Handler[] handlers = rootLogger.getHandlers();
        if(handlers[0] instanceof ConsoleHandler){
            rootLogger.removeHandler(handlers[0]);
        }
        logger.setLevel(Level.INFO);
        fileTxt = new FileHandler("Logging.txt");
        fileHTML = new FileHandler("Logging.html");

        //create a txt formatter
        formatterTxt = new SimpleFormatter();
        fileTxt.setFormatter(formatterTxt);
        logger.addHandler(fileTxt);

        //create an HTML formatter
        formatterHTML = new BotmanHtmlFormatter();
        fileHTML.setFormatter(formatterHTML);
        logger.addHandler(fileHTML);
    }
}
