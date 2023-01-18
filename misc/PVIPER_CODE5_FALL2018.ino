////////////////////////////////////////////////////////////////////////////
//  COURSE:   AAE 41800 Fall 2018                                         //
//  TEAM:     PVIPER                                                      //
//  MEMBERS:  Marten Berlin, Dhruv Jain, Ryan Keith,                      //
//            Osvaldo A. Martin, Brady Walter                             //
//  TITLE:    PVIPER Master Code                                          //
//  CREATED:  November 2018                                               //
//  MODIFIED: Fall 2018                                                   //
//  GUIDE #:  NSPM-MA0002-C  (Sept 12, 2017)                              //
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//                       C O D E   D E S C R I P T I O N                  //
////////////////////////////////////////////////////////////////////////////
/*
  B A S I C S ------------------------------------------------------------
  This code is designed for use with Arduino IDE and the hardware combo of
  the Adafruit Feather M0 Adalogger and the Analog Devices ADIS-16228
  accelerometer along with its breakout board. 
  Please refer to the Feather M0 help page at
  https://bit.ly/2Jdk50P

  L A N G U A G E --------------------------------------------------------
  Due to the use within the Arduino IDE environment, this code is designed
  with the Arduino Programming Language, which itself is base on C++. For
  a better understanding, please visit https://bit.ly/2vEwyIX . Note that
  while the language is based on C++, Arduino IDE is written in Java. If
  editing the code in Notepad++, Sublime, Atom, Brackets, VS Code, etc.,
  setting sytax to either C++ or Java works fine. Java is usually preffered
  because it highlights more of the text for easy reading.

  E D I T O R --------------------------------------------------------------
  The ideal situation is to use Arduino IDE, which can be easily downloaded 
  from the arduino official website. However, atom also works fine as the 
  language is based on C++.

  F U N C T I O N ----------------------------------------------------------
  The code connects the Feather M0 Adalogger and the Analog Devices ADIS16228
  accelerometer through SPI communication. The code begins with the data
  logger checking SD Card functionality followed by a test of the connection
  with the accelerometer. After the accelerometer passes the test checks,
  various settings are changed to enable desired recording mode and outputs.
  The code then begins the looping sequence, recording vibration data only
  when the NanoRacks sends the signal during different parts of space flight.
  The signal from the Nanorack still needs to be worked on. But currently 
  the code records the acceleration in the three directions, running time, 
  and temperature. This data is then saved in a SD card. 
  
  D A T A   S T O R A G E ------------------------------------------------
  General system status readouts are printed to the serial monitor, with
  more important system status messages (such as ready to write, etc.) are
  both printed to the monitor and written to the SD Card. The SD Card is
  also used to store the accelerometer data and inputs from NanoRacks.

  I N P U T S ------------------------------------------------------------
  The system begins running the code and record data automatically once the power
  supply is on.

  O U T P U T S ----------------------------------------------------------
  Manual Time Capture data is stored to the SD Cared to be read by the user
  on a separate system. System timestamp is also recorded in the
  file for reference. For each flight, the code will generate a new file in
  numerical sequence. FFT is not performed by the ADIS16228 as to ensure
  all time is used for recording instead of performing transforms. If FFT is
  desired, either use Manual FFT mode or or perform FFT in a separate program or
  software like Matlabs.

  H E L P ----------------------------------------------------------------
  For further help and documentation, please refer to either the PVIPER
  directory for user manuals or to the Adafruit or Analog Devices product
  sites. There is also an annotated user manual for the ADIS16228 in the
  PVIPER directory which highlights and notes all that appears in the code.
  It is extremely helpful for quick reference if you do not want to read the
  report. Also, if you're ever confused about what a hex value converts to,
  this link is the best I found for conversions http://calc.50x.eu/

////////////////////////////////////////////////////////////////////////////
//                        S Y S T E M   L A Y O U T                       //
////////////////////////////////////////////////////////////////////////////
/*
  ACCEL PIN & DESC.       LINK     LOGGER PIN & DESC.
  --------------------    -----    ------------------------
  10: Reset               >>>>>    6: Reset (#9 on board)
  11: SPI Data Input      >>>>>    23: MOSI
  12: SPI Data Output     >>>>>    22: MISO
  13: SPI Serial Clock    >>>>>    24: Serial Clock
  14: SPI Chip Select     >>>>>    19: Chip Select (A5 on Board)
  15: Digital I/O #1      >>>>>    Digital IO #1 (Unused)
  3,4: Ground             >>>>>    Ground
  1,2: Power Supply       >>>>>    3.3v Power
  5,8: Ground             >>>>>    Ground (Unconnected)
  6,9: N/A                >>>>>    Unconnected
  7: Digial I/O #2        >>>>>    Digital IO #2 (Unused)
*/

////////////////////////////////////////////////////////////////////////////
//                           M A I N   C O D E                            //
////////////////////////////////////////////////////////////////////////////

// CODE SETUP //////////////////////////////////////////////////////////////

  // Library Declaration - - - - - - - - - - - - - - - - - - - - - - -
  #include <SPI.h>          	         // Allows communication with SPI Devices
  #include <SD.h>           	         // Allows usage of SD Cards
 

  // Ready SPI Settings (See ADIS16228 Table 7) - - - - - - - - - - - -
  SPISettings settingsADIS(2250000, MSBFIRST, SPI_MODE3);

	// Device Variables -----------------------------------------------------

  // Nanoracks definition - - - - - - - - - - - - - - - - - - - - - - 
  #define NUMDATAFIELDS   21           // Number of data fields for each packet.
  #define MAXBUFSIZE      2000         // Maximum buffer size for serial packet.
  #define MAXFIELDSIZE    200          // Maximum size of any data field in the serial packet.
  #define SUCCESS         0            // Success return code.
  #define Error           -1           // Error return code.

  // Data Logger Pin Declaration - - - - - - - - - - - - - - - - - - -
  int chipSelect = 4;                  // Pin # for SD Card Select
  int reset      = 6;                  // Pin # for Reset (GPIO #6)
  int ledG       = 8;                  // Pin # for Green LED
  int ledR       = 13;                 // Pin # for Red LED
  int dio1       = 18;                 // Pin # for Interrupt
  int spiSS      = 19;                 // Pin # for Slave Select
  int miso       = 22;                 // Pin # for Master Input / Slave Output
  int mosi       = 23;                 // Pin # for Master Output / Slave Input
  int sclk       = 24;                 // Pin # for Serial Clock
      

  // Accelerometer short Addresses for Regitsters - - - - - - - - - - -
  // (NOTE: "0x" is just indicator that proceeding value is hex)
  byte BUF_PNTR       = 0x10;		       // Pointer to Load Axes Samples
  byte X_BUF          = 0x14;			     // Output Buffer for X-Axis Samples
  byte Y_BUF          = 0x16;			     // Output Buffer for Y-Axis Samples
  byte Z_BUF          = 0x18;			     // Output Buffer for Z-Axis Samples
  byte REC_CTRL1      = 0x1A;          // Recording Control
  byte REC_CTRL2      = 0x1C;          // Recording Control
  byte AVG_CNT        = 0x3A;		       // Sample Rate Counter
  byte DIAG_STAT      = 0x3C;		       // System Status and Error Reports
  byte GLOB_CMD       = 0x3E;		       // Global Command - Various Uses
  byte TEMP_OUT       = 0x08;          // Output Buffer for Temperature
  byte TIME_STAMP_L   = 0x4C;          // Lower Time Buffer
  byte DIO_CTRL       = 0x36;          // For busy read indicator
  byte REC_CNTR       = 0x78;          // Record Counter
  byte SUPPLY_OUT     = 0x0A;          // Record Counter
  byte CAL_ENABLE     = 0x07A;         // Calibration

  // Variable Declaration
  uint16_t spiCheck;                   // The variable store the return 2 bytes value read from accelerometer
  File vibeFile;                       // Global Declaration for file vairable
  String fileName;                     // Global Declaration for file name
  int count;

  short int X_BUFF[512];               // Saves buffer value, acceleration, in X direction, g units, short int because unlike default short as 8 bit, Arduino considers short as 16 bits 
  short int Y_BUFF[512];               // Saves buffer value, acceleration, in X direction, g units, short int because unlike default short as 8 bit, Arduino considers short as 16 bits
  short int Z_BUFF[512];               // Saves buffer value, acceleration, in X direction, g units, short int because unlike default short as 8 bit, Arduino considers short as 16 bits
  uint16_t tempCheck;                  // Saves 16 bit of Temp_out and stores one value for every iteration so for 512 values of accerlation in x,y,z there is only 1 temperature value
  uint16_t timestamp;                  // Saves 16 bit of Time_Stamp_L and stores one value for every iteration so for 512 values of accerlation in x,y,z there is only 1 timestamp value
  float temp;                          // Stores the temperature value after the conversion factor is applied
  
  // Struct to contain all of the NRNSP flight data in one object.
  typedef struct NRdata
  {
    char   flight_state;
    double exptime;
    double altitude;
    double velocity[3];
    double acceleration[3];
    double attitude[3];
    double angular_velocity[3];
    bool   liftoff_warn;
    bool   rcs_warn;
    bool   escape_warn;
    bool   chute_warn;
    bool   landing_warn;
    bool   fault_warn;
 } NRdata;


  void setup(){

    // General -------------------------------------------------------------

    // Open Comm Channels - - - - - - - - - - - - - - - - - - - - - - -
    Serial.begin(115200);           // Opens Serial Communications
    SPI.begin();                    // Opens SPI Communications

    // Configure SPI Pin - - - - - - - - - - - - - - - - - - - - - - - -
    pinMode(spiSS, OUTPUT);			    // Sets Slave Select Pin as Output
    pinMode(mosi, OUTPUT);          // Sets Master Out Slave In Pin as Output
    pinMode(miso, INPUT);           // Sets Master In Slave Out Pin as Input
    pinMode(sclk, OUTPUT);          // Sets Serial Clock Pin as Output
    pinMode(reset, OUTPUT);         // Sets Reset Pin as Output
    pinMode(dio1, INPUT);           // Sets DIO_1 register as Input
    
    digitalWrite(spiSS,HIGH);
    digitalWrite(reset,HIGH);       //Reset pin is on Active low so need to set to high

    // Initialize LEDs for Use - - - - - - - - - - - - - - - - - - - - -
    pinMode(ledG, OUTPUT);          // Initialize Green LED as output
    pinMode(ledR, OUTPUT);          // Initialize Red LED as output

    // Initialize SD Card --------------------------------------------------

    // Wait to Connect to SD Card - - - - - - - - - - - - - - - - - - -
    while (!Serial) { ; }

    // Print Result to Console - - - - - - - - - - - - - - - - - - - - -
    Serial.print("Initializing SD card... ");

    if (!SD.begin(chipSelect)) {

      Serial.println("Card failed or not found");
      return;
    }

    Serial.println("Card found and initialized.");

    // Initialize Data Log File --------------------------------------------

    // Create txt file - - - - - - - - - - - - - - - - - - -
    String fileType = ".txt";

    int i = 0;
    int fileCondition;
    
    do {
      
      fileName = i + fileType;                                                 // Define file name
    
      if (SD.exists(fileName))
      {
        i = i + 1;
        fileCondition = 1;                                                     // If the file exist, i value plus one, and keep running the loop
      } 
      else 
      {
        vibeFile = SD.open(fileName, FILE_WRITE);                              // If the file doesn't exist, create the file in write mode
        fileCondition = 0;                                                     // Kill the loop
      }
    } while(fileCondition);

    // Print and Write File Status - - - - - - - - - - - - - - - - - - -
    if (vibeFile){                                                             // Boolean true if file exists
        
      vibeFile.println();                                                      // Writes blank line
      Serial.println("File initialized. Prepared to write.");
      
      bool sdCheck = true;		                                                 // Used as Check Later
    }
    else {
    
        Serial.println("Error creating or opening vibration data log.");
        
        bool sdCheck = false;		                                               // Used as Check Later
    }

    // Check for Accelerometer ---------------------------------------------
	
    SPI.beginTransaction(settingsADIS);	                                       // Begins Com w/ Accelerometer

    writeCommand(GLOB_CMD, 0x80);                                              // Reset ADIS: 
                                                                               //.. GLOB_CMD[7] = 1; DIN = 1000000
    delay(100);                                                                //All the Delays are important to give sufficient time for IDE and accelerometer to process the commands

    spiCheck = readCommand(DIAG_STAT);   // Read DIAG_STAT value
    
    Serial.println("Checking connection and status of accelerometer...");
    vibeFile.println("Checking connection and status of accelerometer...");
    Serial.println((uint16_t) spiCheck);                                       // Output DIAG_STAT to serial port for debugging and understanding
    if (bitRead(spiCheck, 3)){                                                 // Check for SPI Comm failure
      
      Serial.println("\tSPI communication error w/ accelerometer.");
      vibeFile.println("\tSPI communication error w/ accelerometer.");
    }
    else {

      Serial.println("\tSPI communication w/ accelerometer PASSED.");        
      vibeFile.println("\tSPI communication w/ accelerometer PASSED.");

      // Self-Test Accelerometer ---------------------------------------------
      writeCommand(GLOB_CMD, 0x04);			                                       // ADIS Self Test: 
                                                                               //.. GLOB_CMD[2] = 1
                                                                               //.. DIN = 100
      
     delay(50);                                                                    
                                                                             
      writeCommand(GLOB_CMD, 0x08);                                            // Reset factory settings                              
      delay(100);                                                              
      spiCheck = readCommand(DIAG_STAT);                                       // Read DIAG_STAT value
      timestamp = readCommand(TIME_STAMP_L);                                   // Read Time_stamp value
      Serial.println((uint16_t) spiCheck);                                    // Output DIAG_STAT to serial port for understanding
      if (bitRead(spiCheck, 5)) {                                              // Check for self-test diagnostic error

        Serial.println("\tAccelerometer self-test diagnostic error"); 
        vibeFile.println("\tAccelerometer self-test diagnostic error");
      }
      else {

        Serial.println("\tAccelerometer self-test diagnostic PASSED.");
        vibeFile.println("\tAccelerometer self-test diagnostic PASSED.");

        // Setup Accelerometer -------------------------------------------------

        // Print Status - - - - - - - - - - - - - - - - - - - - - - - - - - -
        Serial.println("Assigning accelerometer settings...");
        
        

        // Recording Mode - - - - - - - - - - - - - - - - - - - - - - - - -
        writeCommand(REC_CTRL1, 0x1102);		                                    //REC_CTRL1 set to mode 2 of SR0 , Sets Manual Time Capture Mode  and Turns on SR0 only:
                                                                                //mode set in AVG_CNT 
        writeCommand(REC_CTRL2, 0xF0);                                         
                                                                                //.. REC_CTRL1[1:0] = 01 and REC_CTRL1[10] = 1
                                                                                //.. DIN = 000001000000000
        delay(20);
        
        writeCommand(DIO_CTRL,0x0F);                                            //Set DIO1 line for busy/data line
        delayMicroseconds(20);                                                  // delay
        writeCommand(AVG_CNT,0x02);                                             //Set mode 2 of SR0
        delay(50);
        writeCommand(CAL_ENABLE,0x04);                                          //Calibration Enable to decrease the noise
        delay(100);
        
        spiCheck = readCommand(CAL_ENABLE);                                       // Read CAL_ENABLE value
        Serial.println((uint16_t) spiCheck);                                   

        
        // Print Result - - - - - - - - - - - - - - - - - - - - - - - - - - -
        vibeFile.println("System ready for vibration recording.");
        vibeFile.println("X_acce Y_acce Z_acce time Temp");
        Serial.println("System ready for vibration recording.");
             }
      
    }
    
    // Close SD -----------------------------------------------------
    vibeFile.close();		                                                         //closes File
   
  }

void loop(){

                                        
    // Connection with nanorack - - - - - - - - - - - - - - - - - - - - - - - -
  char buffer[MAXBUFSIZE + 1] = { 0};                                          // Buffer for receiving serial packets.
  int res;                                                                     // Value for storing results of function calls.
 
  NRdata flight_info;                                                          // Struct for holding current flight data.
  
  memset(&flight_info, 0, sizeof(NRdata));                                     // Initialize the struct and all its data to 0
  
  res = Serial.readBytes(buffer, MAXBUFSIZE);                                  // Read in the serial data up to the maximum serial packet size
  
  buffer[res] = 0;                                                             // Null terminate the buffer
  res = parse_serial_packet(buffer, &flight_info);                             // Update the current flight info with the new data
  
    // Record Data ---------------------------------------------------------
	    
  //if(flight_info.flight_state == 'A' | 'B' | 'C' | 'F' | 'G' | 'H') {         // If want to record the data based on flight status, put condition in: flight_info.flight_state == 'A' | 'B' | 'C' | 'F' | 'G' | 'H'
    configSPI();
    delay(20);
    
    writeCommand(GLOB_CMD, 0x800);                                              // Start manual capture sequence
   
    delay(250);                   
    
    dataRead();
    timestamp = readCommand(TIME_STAMP_L);                                       // Read time value - lower byte only
    tempCheck = readCommand(TEMP_OUT);                                          // Read TEMP_OUT value
    temp = (1278 - tempCheck)*0.47 + 25;                                       // Calibrate TEMP_OUT to get temperature in degrees, deduced from the ADIS16228- guide
    
    //File in Write Mode
    vibeFile = SD.open(fileName, FILE_WRITE);
    delay(20);
    
    // In Manual Time Capture Mode, there are 512 data in one run so we save the 512 acceleration by first convertin into integer and 
    // multiplying by the sensitivity factor. However, there is only one value for temperature for 512 values of accerlation                  
    
   for (int i = 0; i < 512; i++) {

      // Printing to file for output
      vibeFile.print((short int) X_BUFF[i]*0.000030518);                                    // Multiply by accel sensitivity
      vibeFile.print(" ");
      vibeFile.print((short int) Y_BUFF[i]*0.000030518);                                    // Multiply by accel sensitivity
      vibeFile.print(" ");
      vibeFile.print((short int) Z_BUFF[i]*0.000030518);                                    // Multiply by accel sensitivity
      vibeFile.print(" ");
      vibeFile.print(timestamp);                                                            // TIME_STAMP_L value
      vibeFile.print(" ");
      vibeFile.println(temp);                                                               // calibrated TEMP_OUT 
      //Serial.println((short int) X_BUFF[i]*0.000030518);                                    // To review important values in the Serial Monitor
       
    }
    
 
	// Close SD and End SPI Communication once flight finished -----------------
  
  vibeFile.close();                                                                       // Closes File
   
}



////////////////////////////////////////////////////////////////////////////
//                     C U S T O M   F U N C T I O N S                    //
////////////////////////////////////////////////////////////////////////////


// writeCommand //////////////////////////////////////////////////////////
// Refenced from the Teensy code, link in the meeting notes
// Writes one byte of data to the specified register over SPI.
// regAddr - address of register to be written
// regData - data to be written to the register
// Copyright: This function is copied, from ADIS16228_Teensy_Example.ino, written by Juan Jose Chong <juan.chong@analog.com> with lot of modification with the help of Dr. Anthony Cofer
////////////////////////////////////////////////////////////////////////////

void writeCommand(byte regAddr, uint16_t regData) {

  // Sanity-check address and register data
  
  uint16_t addr     = (((regAddr & 0x7F) | 0x80) << 8);           // Toggle sign bit, and check that the address is 8 bits
  uint16_t lowWord  = (addr | (regData & 0xFF));                  // OR Register address (A) with data(D) (AADD)
  uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)); // OR Register address with data and increment address
  
  // Split words into chars for 8-bit transfers (Arduino language limitation)
  
  uint8_t highBytehighWord  = (highWord >> 8);
  uint8_t lowBytehighWord   = (highWord & 0xFF);
  uint8_t highBytelowWord   = (lowWord >> 8);
  uint8_t lowBytelowWord    = (lowWord & 0xFF);
  
  // Write highWord to SPI bus
  
  digitalWrite(spiSS, LOW);                                       // Set CS low to enable device
  SPI.transfer(highBytehighWord);                                 // Write high byte from high word to SPI bus
  SPI.transfer(lowBytehighWord);                                  // Write low byte from high word to SPI bus
  digitalWrite(spiSS, HIGH);                                      // Set CS high to disable device

  delayMicroseconds(20);                                        // Delay to not violate read rate

  // Write lowWord to SPI bus
  
  digitalWrite(spiSS, LOW);                                       // Set CS low to enable device
  SPI.transfer(highBytelowWord);                                  // Write high byte from low word to SPI bus
  SPI.transfer(lowBytelowWord);                                   // Write low byte from low word to SPI bus
  digitalWrite(spiSS, HIGH);                                      // Set CS high to disable device

  delayMicroseconds(20);                                        // Delay to not violate read rate;
}

// readCommand ///////////////////////////////////////////////////////////
// Refenced from the Teensy code, link in the meeting notes
// Reads two 8-bit bytes (one word) in two sequential registers over SPI
// regAddr - address of register to be read
// return - (int) signed 16 bit 2's complement number
// Copyright: This function is copied, from ADIS16228_Teensy_Example.ino, written by Juan Jose Chong <juan.chong@analog.com> with lot of modification with the help of Dr. Anthony Cofer
////////////////////////////////////////////////////////////////////////////////////////////

uint16_t readCommand(byte regAddr) {

  // Write register address to be read
  
  digitalWrite(spiSS, LOW);                                       // Set CS low to enable device
  SPI.transfer(regAddr);                                          // Write address over SPI bus
  SPI.transfer(0x00);                                             // Write 0x00 to the SPI bus fill the 16 bit transaction requirement
  digitalWrite(spiSS, HIGH);                                      // Set CS high to disable device
  delayMicroseconds(20);                                          // Delay to not violate read rate

  // Read data from requested register
  
  digitalWrite(spiSS, LOW);                                       // Set CS low to enable device
  uint8_t _msbData = SPI.transfer(0x00);                          // Send (0x00) and place upper byte into variable
  uint8_t _lsbData = SPI.transfer(0x00);                          // Send (0x00) and place lower byte into variable
  digitalWrite(spiSS, HIGH);                                      // Set CS high to disable device
  delayMicroseconds(20);                                          // Delay to not violate read rate
  int16_t _dataOut = (_msbData << 8) | (_lsbData & 0xFF);         // Concatenate upper and lower bytes
  
  // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.
  
  return(_dataOut);
}

// readFFT ///////////////////////////////////////////////////////////////
// Reads all 512 bins from the sensor.
// regAddr - address of the buffer to be read
// Returns a pointer to an array of sensor data.
// *NOTE* This function only works with X_BUF, Y_BUF, and Z_BUF!
// Copyright: This function is copied, with minimum modification from ADIS16228_Teensy_Example.ino, written by Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////
// No inputs required.
////////////////////////////////////////////////////////////////////////////

void readFFT(uint8_t regAddr, short int fftdata[512]) {                                //TC set fixed array
   
  for (int i = 0; i < 512; i++){
     
    fftdata[i] = readCommand(regAddr);
  }  
 
}

// dataRead /////////////////////////////////////////////////////////////////
// Calls readFFT function to store acceleration in array of respective direction 
////////////////////////////////////////////////////////////////////////////
// No inputs required.
////////////////////////////////////////////////////////////////////////////

void dataRead(){
    readFFT(X_BUF, X_BUFF);                                                    // Read data from buffer
    delay(20);                                                                 // Important delay
    readFFT(Y_BUF, Y_BUFF);                                                    // Read data from buffer
    delay(20);                                                                 // Important delay
    readFFT(Z_BUF, Z_BUFF);                                                    // Read data from buffer
    delay(20);                                                                 // Important delay
    
}


// configSPI ////////////////////////////////////////////////////////////////
// Configure SPI before reading data if SD card file has been open or closed
// Returns an integer value 1 to show that the function ran successfully
////////////////////////////////////////////////////////////////////////////
// No inputs required.
////////////////////////////////////////////////////////////////////////////

int configSPI() {
  SPISettings settingsADIS(2250000, MSBFIRST, SPI_MODE3);
  SPI.beginTransaction(settingsADIS);
  return(1);
}

// parse_serial_packet /////////////////////////////////////////////////////
// Takes in a buffer containing an NRNSP serial data packet and parses the information into the
// provided data struct.
// buf            =   Buffer containing serial data packet.
// flight_data    =   Struct containing current flight data.
// return   =   SUCCESS or ERROR
// Copyright: This function is copied, with minimum modification from the example provided by Nanoracks
/////////////////////////////////////////////////////////////////////////////

int parse_serial_packet(const char* buf, NRdata* flight_data){

  int res;                          // Value for storing result of function calls.
  int fieldnum = 1;                 // Index for field number being parsed.
  int index = 0;                    // Index for position in serial buffer.
  char temp[MAXFIELDSIZE] = { 0 };  // Temporary buffer for holding a data field.

  // If the buffer is empty then return with an error.
  if (strlen(buf) == 0)
  {
    return (Error);
  }

  // Continue parsing until the end of the buffer is reached
  while (fieldnum != (NUMDATAFIELDS+1))
  {
    // Scan the buffer from the current index until the next comma and place the data into the
    // temporary buffer.
    res = sscanf((buf + index), "%[^,]", temp);

    // If sscanf failed to get a parameter then the buffer was not in the correct format so the
    // function should return with an error.
    if (res == 0)
    {
      return (Error);
    }

    // Increment the buffer index by the length of the data field plus the comma.
    index += (strlen(temp) + 1);

    // Depending upon the current data field being parsed, convert the data in the temporary buffer
    // to the appropriate format and store it in the flight data struct.
    switch (fieldnum)
    {
      case 1:
        // Flight state is just a single character.
        flight_data->flight_state = temp[0];
        break;
      case 2:
        // atof() converts a c-string to a floating point number.
        flight_data->exptime = atof(temp);
        break;
      case 3:
        flight_data->altitude = atof(temp);
        break;
      case 4:
        flight_data->velocity[0] = atof(temp);
        break;
      case 5:
        flight_data->velocity[1] = atof(temp);
        break;
      case 6:
        flight_data->velocity[2] = atof(temp);
        break;
      case 7:
        flight_data->acceleration[0] = atof(temp);
        break;
      case 8:
        flight_data->acceleration[1] = atof(temp);
        break;
      case 9:
        flight_data->acceleration[2] = atof(temp);
        break;
      case 10:
        flight_data->attitude[0] = atof(temp);
        break;
      case 11:
        flight_data->attitude[1] = atof(temp);
        break;
      case 12:
        flight_data->attitude[2] = atof(temp);
        break;
      case 13:
        flight_data->angular_velocity[0] = atof(temp);
        break;
      case 14:
        flight_data->angular_velocity[1] = atof(temp);
        break;
      case 15:
        flight_data->angular_velocity[2] = atof(temp);
        break;
      case 16:
        // For the flight warnings, assign a boolean value of false for a 0 and true for anything
        // else.
        flight_data->liftoff_warn = (temp[0] == '0') ? false : true;
        break;
      case 17:
        flight_data->rcs_warn = (temp[0] == '0') ? false : true;
        break;
      case 18:
        flight_data->escape_warn = (temp[0] == '0') ? false : true;
        break;
      case 19:
        flight_data->chute_warn = (temp[0] == '0') ? false : true;
        break;
      case 20:
        flight_data->landing_warn = (temp[0] == '0') ? false : true;
        break;
      case 21:
        flight_data->fault_warn = (temp[0] == '0') ? false : true;
        break;
    }

    // Increment the index for the current data field.
    fieldnum++;
  }
  return SUCCESS;
}
