// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimpleCSVLogger2194 {

    long log_write_index;
    String log_name = null;
    String output_dir = "C:"; // USB drive is mounted to /U on roboRIO
    BufferedWriter log_file = null;
    public boolean log_open = false;
    int numberOfElements;
    double startTime = 0.;
    String uniqueID;

    /**
     * Determines a unique file name, and opens a file in the data captures
     * directory and writes the initial lines to it.
     * 
     * @param data_fields  A set of strings for signal names to write into the file
     * @param units_fields A set of strings for signal units to write into the file
     * @return 0 on successful log open, -1 on failure
     */
    public int init(String name, String data_fields, String units_fields) {

        SmartDashboard.putString("CSVName", name);

        // double temp = (int)Timer.getFPGATimestamp();
        startTime = Timer.getFPGATimestamp();
        // uniqueID = String.valueOf(temp);

        if (log_open) {
            System.out.println("Warning - log is already open!");

            return 0;
        }

        SD.putN2("CSV1T", Timer.getFPGATimestamp() - startTime);
        File file = new File(output_dir);
        if (!file.exists()) {

            SD.putN2("CSV2T", Timer.getFPGATimestamp() - startTime);
            if (file.mkdir()) {
                System.out.println("Directory is created!");
                SD.putN2("CSV3T", Timer.getFPGATimestamp() - startTime);
            } else {
                System.out.println("Failed to create directory!");
                SD.putN2("CSV4T", Timer.getFPGATimestamp() - startTime);
            }
        }

        log_open = false;

        System.out.println("Initalizing Log file...");
        // numberOfElements = data_fields.length;
        try {
            // Reset state variables
            log_write_index = 0;
            SD.putN2("CSV5T", Timer.getFPGATimestamp() - startTime);
            // Determine a unique file name
            // log_name = output_dir + "log_" + name + ".csv";
            SD.putN2("CSV6T", Timer.getFPGATimestamp() - startTime);
            // Open File
            FileWriter fstream = new FileWriter(name, true);
            log_file = new BufferedWriter(fstream);
            SD.putN2("CSV7T", Timer.getFPGATimestamp() - startTime);
            SD.putN2("CSV8T", Timer.getFPGATimestamp() - startTime);
            log_file.write(data_fields);

            log_file.write(units_fields);
            SD.putN2("CSV9T", Timer.getFPGATimestamp() - startTime);

        }
        // Catch ALL the errors!!!
        catch (IOException e) {
            System.out.println("Error initializing log file: " + e.getMessage());

            return -1;
        }
        SD.putN2("CSV10T", Timer.getFPGATimestamp() - startTime);
        System.out.println("done!");
        log_open = true;
        return 0;

    }

    /**
     * Write a list of doubles to the output file, assuming it's open. Creates a new
     * line in the .csv log file.
     * 
     * @param data_elements Values to write (any number of doubles, each as its own
     *                      argument). Should have the same number of arguments here
     *                      as signal names/units set during the call to init()
     * @return 0 on write success, -1 on failure.
     */
    public int writeData(double... data_elements) {
        String line_to_write = "";

        if (log_open == false) {
            System.out.println("Error - Log is not yet opened, cannot write!");
            return -1;
        }

        try {

            // Write user-defined data
            String comma = ",";
            int pass = 0;
            for (double data_val : data_elements) {
                if (pass == numberOfElements - 1)
                    comma = "";
                else
                    comma = ",";
                pass++;
                data_val = Math.round(data_val * 1000.) / 1000.;
                if (Math.abs(data_val) < .0001)
                    data_val = 0;
                line_to_write = line_to_write.concat(Double.toString(data_val) + comma);
            }

            // End of line
            line_to_write = line_to_write.concat("\n");

            // write constructed string out to file
            log_file.write(line_to_write);

        }
        // Catch ALL the errors!!!
        catch (IOException e) {
            System.out.println("Error writing to log file: " + e.getMessage());
            return -1;
        }

        log_write_index++;
        return 0;
    }

    /**
     * Clears the buffer in memory and forces things to file. Generally a good idea
     * to use this as infrequently as possible (because it increases logging
     * overhead), but definitely use it before the roboRIO might crash without a
     * proper call to the close() method (during brownout, maybe?)
     * 
     * @return Returns 0 on flush success or -1 on failure.
     */
    public int forceSync() {
        if (log_open == false) {
            System.out.println("Error - Log is not yet opened, cannot sync!");
            return -1;
        }
        try {
            log_file.flush();
        }
        // Catch ALL the errors!!!
        catch (IOException e) {
            System.out.println("Error flushing IO stream file: " + e.getMessage());
            return -1;
        }

        return 0;

    }

    /**
     * Closes the log file and ensures everything is written to disk. init() must be
     * called again in order to write to a new file.
     * 
     * @return -1 on failure to close, 0 on success
     */
    public int close() {

        if (log_open == false) {
            System.out.println("Warning - Log is not yet opened, nothing to close.");

            return 0;
        }

        try {
            log_file.close();
            log_open = false;
        }
        // Catch ALL the errors!!!
        catch (IOException e) {
            System.out.println("Error Closing Log File: " + e.getMessage());
            SD.putN2("CSV7", 6);
            return -1;
        }
        return 0;

    }

}