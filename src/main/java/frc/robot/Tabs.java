package frc.robot;

import java.util.HashMap;
import java.util.HashSet;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class Tabs {
    private static HashMap<String, HashSet<String>> tabNames = new HashMap<>();
    private static HashMap<String, SimpleWidget> widgets = new HashMap<>();

     // Just creates a new tab.
     public static ShuffleboardTab addTab(String tabName){
          ShuffleboardTab tab = Shuffleboard.getTab(tabName);
          if(!tabNames.containsKey(tabName)) tabNames.put(tabName, new HashSet<>());
          return tab;
     }

     //Creates a standardized system of internal naming. We don't want these names to repeat.
     private static String getEntryName(String tabName, String entryName){
          return tabName + "_" + entryName;
     }

     //When you call this, you need to make sure you create the tab in the first place. Otherwise, it'll crash
    public static SimpleWidget putNumber(String tabName, String label, double number){
          HashSet<String> names;
          if(tabNames.containsKey(tabName)) names = tabNames.get(tabName);
          else return null;//<-- TODO: Need a better solution
          /*
           * Shuffleboard gets mad at you if you try to create 2 widgets with the same name.
           * This means that if you try to create a widget when one already exists, the program will crash.
           * This if-statement gets around that
           */
          String newName = getEntryName(tabName, label);
          if(names.contains(label)){
               SimpleWidget widget = widgets.get(newName);
               widget.getEntry().setDouble(number);
               return widget;
          }

          names.add(label);
          
          SimpleWidget widget = addTab(tabName).addPersistent(label, 0);
          widget.getEntry().setDouble(number);
          widgets.put(newName, widget);
          return widget;
    }

     public static double getNumber(String tabName, String label){
          HashSet<String> names = tabNames.get(tabName);
          String newName = getEntryName(tabName, label);
          if(names.contains(label)){
               return widgets.get(newName).getEntry().getDouble(0);
          }
          return -0.001768;
     }

     public static SimpleWidget putBoolean(String tabName, String label, boolean bool){
          HashSet<String> names;
          if(!tabNames.containsKey(tabName)) addTab(tabName);
          names = tabNames.get(tabName);
          
          String newName = getEntryName(tabName, label);
          DriverStation.reportError("AHH " + names.contains(label) + "******************", false);
          DriverStation.reportError(widgets.toString(), bool);
          if(names.contains(label)){
               SimpleWidget widget = widgets.get(newName);
               widget.getEntry().setBoolean(bool);
               return widget;
          }

          names.add(label);

          SimpleWidget widget = addTab(tabName).add(label, false);
          widget.getEntry().setBoolean(bool);
          widgets.put(newName, widget);
          return widget;
     }

     public static boolean getBoolean(String tabName, String label){
          HashSet<String> names = tabNames.get(tabName);
          String newName = getEntryName(tabName, label);
          if(names.contains(label)){
               return widgets.get(newName).getEntry().getBoolean(false);
          }
          return false;
     }
}
