package frc.robot;

import java.util.HashSet;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class Tabs {
    private static HashSet<String> tabNames;
    private static ShuffleboardTab lastTab; //<-- TODO: Test to see if this actually works

    public static ShuffleboardTab addTab(String tabName){
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        tabNames.add(tabName);
        lastTab = tab;
        return tab;
    }

    public static SimpleWidget putNumber(String tabName, String label, double number){
        return Shuffleboard.getTab(tabName).add(label, number);
    }

    // public static void putNumber(String label, double number){
    //     if(!tabNames.contains(lastTab.getTitle())) return;
    //     lastTab.add(label, number);
    // }

   public static double getNumber(String tabname, String label){
        GenericEntry entry = Shuffleboard.getTab("tabName").add("label", 0).getEntry();
        return entry.getDouble(0);
   }

   public static SimpleWidget putBoolean(String tabName, String label, boolean bool){
        return Shuffleboard.getTab(tabName).add(label, bool);
   }

   public static boolean getBool(String tabName, String label){
        GenericEntry entry = Shuffleboard.getTab("tabName").add("label", 0).getEntry();
        return entry.getBoolean(false);
   }
}
