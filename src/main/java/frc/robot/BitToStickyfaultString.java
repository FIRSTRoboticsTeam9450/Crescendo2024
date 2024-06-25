package frc.robot;

public class BitToStickyfaultString {
    private static String[] stickyFaultStringRevMotor = {"Brownout", "Over Current", "Watchdog Reset", "Motor Type Fault", 
                             "Sensor Fault", "Stall", "EEPROM Fault", "CAN TX Fault", "CAN RX Fault", 
                             "Has Reset", "Gate Driver Fault", "Other Fault", "Soft Limit Forward", 
                             "Soft Limit Reverse", "Hard Limit Forward", "Hard Limit Reverse"};
    private static String[] stickyFaultStringRevPDH = {"Brownout", "CAN Warning", "Hardware", "CAN Bus Off", 
                             "Firmware", "Has Reset", "CSA Channel Invalid"};

    private static String motorName = "";

    // compare 2^0 all the way to 2^15 b/c short is 16 bits, 0 indexed
    private static boolean hasFault = false;
    public static void getStickyFaultStringRevMotor(short stickyFaults, String motor) {
        motorName = motor;
        hasFault = false;
        int comparedTo = 1;
        for (int i = 0; i < 16; i++) { 
            if ((short) ((int) stickyFaults & (int) comparedTo) != 0) {
                // something triggered at this index
                System.out.println(motorName + " -- STICKY FAULT: " + stickyFaultStringRevMotor[i]);
                hasFault = true;
            } else {
                
            }
            comparedTo = comparedTo << 1; // shifts decimal point to the right (when we compare it is treated as binary)
        }
        if (!hasFault) {
            System.out.println(motorName + " -- NO STICKY FAULTS");
        }
    }

    public static void getStickyFaultStringRevPDH(short stickyFaults) {
        hasFault = false;
        int comparedTo = 1;
        for (int i = 0; i < 7; i++) { 
            if ((short) ((int) stickyFaults & (int) comparedTo) != 0) {
                // something triggered at this index
                System.out.println(motorName + " -- STICKY FAULT: " + stickyFaultStringRevPDH[i]);
                hasFault = true;
            } else {
                
            }
            comparedTo = comparedTo << 1; // shifts decimal point to the right (when we compare it is treated as binary)
        }
        if (!hasFault) {
            System.out.println("PDH -- NO STICKY FAULTS");
        }
    }

    public static void getStickyFaultStringKrakens(int stickyFaultField) {
        if (stickyFaultField == 0) {
            System.out.println("Kraken -- STICKY FAULT: NO STICKY FAULTS");
        } else {

        }


    }


    public static void getStickyFaultString(int stickyFaults, String name, String[] allFaultsAsStrings) {
        motorName = name;
        hasFault = false;
        int comparedTo = 1;
        for (int i = 0; i < allFaultsAsStrings.length - 1; i++) { 
            if (((int) stickyFaults & (int) comparedTo) != 0) {
                // something triggered at this index
                System.out.println(motorName + " -- STICKY FAULT: " + allFaultsAsStrings[i]);
                hasFault = true;
            } else {
                
            }
            comparedTo = comparedTo << 1; // shifts decimal point to the right (when we compare it is treated as binary)
        }
        if (!hasFault) {
            System.out.println(motorName + " -- NO STICKY FAULTS");
        }
    }

    


    public static boolean getHasFault() {
        return hasFault;
    }

    public static String getMotorName() {
        return motorName;
    }

    

}