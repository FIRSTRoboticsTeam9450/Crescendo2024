package frc.robot;

public class BitToStickyfaultString {
    private static String[] stickyFaultString = {"Brownout", "Over Current", "Watchdog Reset", "Motor Type Fault", 
                             "Sensor Fault", "Stall", "EEPROM Fault", "CAN TX Fault", "CAN RX Fault", 
                             "Has Reset", "Gate Driver Fault", "Other Fault", "Soft Limit Forward", 
                             "Soft Limit Reverse", "Hard Limit Forward", "Hard Limit Reverse"};

    // compare 2^0 all the way to 2^15 b/c short is 16 bits, 0 indexed
    public static void getStickyFaultString(short stickyFaults) {
        int comparedTo = 1;
        for (int i = 0; i < 16; i++) { 
            if ((short) ((int) stickyFaults & (int) comparedTo) != 0) {
                // something triggered at this index
                System.out.println("FAULT: " + stickyFaultString[i]);
            } else {

            }
            comparedTo = comparedTo << 1; // shifts decimal point to the right (when we compare it is treated as binary)
        }
    }

}
