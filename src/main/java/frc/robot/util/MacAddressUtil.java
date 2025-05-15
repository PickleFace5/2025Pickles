package frc.robot.util;

import java.net.NetworkInterface;
import java.util.Enumeration;

public class MacAddressUtil {
  public static final String LEVIATHAN_MAC = "00-1A-2C-56-7D-4F";
  public static final String METAL_MELODY_MAC = "00-80-2F-38-8D-70";
  public static final String LARRY_MAC = "27-0C-FE-C4-66-05";

  @SuppressWarnings("CatchAndPrintStackTrace")
  public static String getMACAddress() {
    try {
      Enumeration<NetworkInterface> networkInterface = NetworkInterface.getNetworkInterfaces();
      StringBuilder macAddress = new StringBuilder();
      while (networkInterface.hasMoreElements()) {
        NetworkInterface tempInterface =
            networkInterface
                .nextElement(); // Instantiates the next element in our network interface
        if (tempInterface != null) {
          byte[] mac =
              tempInterface.getHardwareAddress(); // Reads the MAC address from our NetworkInterface
          if (mac != null) {
            for (int i = 0; i < mac.length; i++) {
              // Formats our mac address by splitting it into two-character segments and hyphenating
              // them
              // (unless it is the final segment)
              macAddress.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
            }
            return macAddress.toString();
          } else {
            System.out.println("Address not accessible");
          }
        } else {
          System.out.println("Network Interface for the specified address not found");
        }
      }
    } catch (Exception e) {
      e.printStackTrace();
    }

    return "";
  }
}
