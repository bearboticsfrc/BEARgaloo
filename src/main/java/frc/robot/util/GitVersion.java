package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.text.SimpleDateFormat;
import java.util.Date;

/** GitVersion class borrowed from team 2832, but cleaned up */
public class GitVersion implements Serializable {
  public String lastCommit;
  public boolean isModified;
  public Date buildDate;
  public String buildAuthor;

  public static GitVersion fromMetadata(
      String lastCommit, boolean isModified, Date buildDate, String buildAuthor) {
    GitVersion version = new GitVersion();

    version.lastCommit = lastCommit;
    version.isModified = isModified;
    version.buildDate = buildDate;
    version.buildAuthor = buildAuthor;

    return version;
  }

  public String getLastCommit() {
    return lastCommit;
  }

  public boolean isModified() {
    return isModified;
  }

  public Date getBuildDate() {
    return buildDate;
  }

  public String getBuildAuthor() {
    return buildAuthor;
  }

  public void printVersions() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SW Version");

    String theBuildDate = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss").format(buildDate);
    table.getEntry("Build Date").setString(theBuildDate);
    table.getEntry("Build Author").setString(buildAuthor);
    table.getEntry("Current Commit").setString(lastCommit);
    table.getEntry("Modified").setBoolean(isModified);

    DataLogManager.log("Build Date:" + theBuildDate);
    DataLogManager.log("Build Author:" + buildAuthor);
    DataLogManager.log("Current commit:" + lastCommit);
    DataLogManager.log("Modified:" + isModified);
  }

  public static GitVersion loadVersion() {
    var path = Filesystem.getDeployDirectory() + "/gitinfo.obj";
    GitVersion version;

    try {
      FileInputStream fileInputStream = new FileInputStream(path);
      ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream);
      version = (GitVersion) objectInputStream.readObject();
      objectInputStream.close();
    } catch (Exception e) {
      // generic catch is usually bad, but here we are using it to create a default whenever there
      // is an issue loading it
      version = GitVersion.fromMetadata("Unknown", false, new Date(), "Unknown");
    }

    return version;
  }

  private static String executeCommand(String command, Runtime runtime) {
    Process process;

    try {
      process = runtime.exec(command);
      process.waitFor();

      return new String(process.getInputStream().readAllBytes());
    } catch (InterruptedException | IOException exc) {
      System.out.println(exc);
      System.exit(1);

      return ""; // Practically will never be reached, compiler doesn't know this though
    }
  }

  private static void writeObject(String location, Object object) {
    try {
      FileOutputStream file = new FileOutputStream("src/main/deploy/gitinfo.obj");
      ObjectOutputStream objectStream = new ObjectOutputStream(file);

      objectStream.writeObject(object);
      objectStream.close();
    } catch (IOException exc) {
      System.exit(1);
      return;
    }
  }

  // this main function should only be called from Gradle
  public static void main(String[] args) {
    Runtime runtime = Runtime.getRuntime();
    Date buildDate = new Date();
    String stdout;

    // get the user who made the commit
    stdout = executeCommand("git config user.name", runtime);
    String buildAuthor = stdout.replace("\n", "");

    // run git log to get the last commits hash
    stdout = executeCommand("git rev-parse --short HEAD", runtime);
    String lastCommit = stdout.replace("\n", "");

    // get the status to see if any files have changed
    stdout = executeCommand("git status -s", runtime);
    boolean isModified = stdout.length() > 0;

    // write object file
    GitVersion version = GitVersion.fromMetadata(lastCommit, isModified, buildDate, buildAuthor);
    writeObject("src/main/deploy/gitinfo.obj", version);

    System.exit(0);
  }
}
