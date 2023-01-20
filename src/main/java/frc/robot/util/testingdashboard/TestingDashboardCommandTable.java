package frc.robot.util.testingdashboard;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Hashtable;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestingDashboardCommandTable {
  Hashtable<String, ArrayList<CommandBase>> table;
  public TestingDashboardCommandTable() {
    table = new Hashtable<String, ArrayList<CommandBase>>();
  }
  public void add(String str, CommandBase cmd) {
    if (table.containsKey(str)) {
      ArrayList<CommandBase> l = table.get(str);
      l.add(cmd);
    } else {
      ArrayList<CommandBase> l = new ArrayList<CommandBase>();
      l.add(cmd);
      table.put(str, l);
    }
  }

  public Enumeration<String> getCommandGroups() {
    return table.keys();
  }

  public ArrayList<CommandBase> getCommandList(String cmdGrgName) {
    return table.get(cmdGrgName);
  }
}
