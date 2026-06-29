package org.frc5010.common.config.json.devices;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;

public class OrchestraParser {
  public static void parse(String robotDirectory) {
    try {
      File directory = new File(Filesystem.getDeployDirectory(), robotDirectory + "/subsystems");
      File deviceFile = new File(directory, "orchestra.json");
      if (!deviceFile.exists()) {
        return;
      }
      OrchestraConfigJson orchestraConfig =
          new ObjectMapper().readValue(deviceFile, OrchestraConfigJson.class);
      orchestraConfig.configure();
    } catch (IOException e) {
      DriverStation.reportError(
          "Error reading Orchestra configuration: " + e.getMessage(), e.getStackTrace());
      return;
    }
  }
}
