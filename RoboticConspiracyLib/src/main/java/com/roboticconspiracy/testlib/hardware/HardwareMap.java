package com.roboticconspiracy.testlib.hardware;

import android.annotation.SuppressLint;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

@SuppressLint("NewApi")
public class HardwareMap {
    private HashMap<String, HashMap<String, HardwareDevice>> hardware = new HashMap<String, HashMap<String, HardwareDevice>>();
    public HardwareMap(String hardwareMapFilePath) throws IOException {
        List<String> lines = Files.readAllLines(Paths.get(hardwareMapFilePath));
        for (int i = 0; i < lines.size(); i++){
            String[] data = lines.get(0).split("|");
            String className = data[0].replace("class:", "").trim();
            String name = data[1].replace("name", "").trim();
            if (!hardware.containsKey(className)){

                hardware.put(className, new HashMap<String, HardwareDevice>());
            }
            if(!name.isEmpty()){
                hardware.get(className).put(name, null);
            } else {
                throw new IllegalNameException();
            }
        }
    }


}
