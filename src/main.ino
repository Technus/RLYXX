#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "SerialState.hpp"

//For unique identification
#define HW_NAME "RLYXX Relay controller, ver. 1.1.1, 09/11/2021"
#define HW_MAN "https://github.com/Technus/RLYXX"
#define HW_ID "RLYXX"
#define HW_SER "0"

const u8 STRING_PRODUCT[] PROGMEM = HW_NAME;
const u8 STRING_MAN[] PROGMEM = HW_MAN;
const u8 STRING_SER[] PROGMEM = HW_SER;

#define HWSER (F(HW_SER))
#define HWID (F(HW_ID))
#define FWVER (F(HW_NAME))

#define USE_SERIAL_STATE

#define CMD_REGISTER (0)
#define RLY_REGISTER (1)
#define BOARD_ADDR_MIN (56)
#define BOARD_COUNT_MAX (8)

#define USB_RLY_TMO 25
#define RLY_TMO 25
#define I2C_TMO 25

#define I2C_BUS (0)
#define I2C_COUNT (1)

//Wire

#define SOFTWARE_I2C_BUS (I2C_BUS + I2C_COUNT)
#define SOFTWARE_I2C_COUNT (0)

//Something

#define SOFTWARE_SERIAL_BUS (SOFTWARE_I2C_BUS + SOFTWARE_I2C_COUNT)
#define SOFTWARE_SERIAL_COUNT (7)

static SoftwareSerial softwareSerialsHolder[SOFTWARE_SERIAL_COUNT] = {
    SoftwareSerial(8, 4),   //7 5
    SoftwareSerial(9, 5),   //6 6
    SoftwareSerial(14, 20), //2 7
    SoftwareSerial(15, 21), //3 8
    SoftwareSerial(16, 17),
    SoftwareSerial(11, 7),
    SoftwareSerial(10, 6),
};

static SoftwareSerial *softwareSerials[SOFTWARE_SERIAL_COUNT] = {
    &softwareSerialsHolder[0],
    &softwareSerialsHolder[1],
    &softwareSerialsHolder[2],
    &softwareSerialsHolder[3],
    &softwareSerialsHolder[4],
    &softwareSerialsHolder[5],
    &softwareSerialsHolder[6],
};

#define HARDWARE_SERIAL_BUS (SOFTWARE_SERIAL_BUS + SOFTWARE_SERIAL_COUNT)
#define HARDWARE_SERIAL_COUNT (1)

static HardwareSerial *hardwareSerials[HARDWARE_SERIAL_COUNT] = {&Serial1};

#define BUS_COUNT (HARDWARE_SERIAL_BUS + HARDWARE_SERIAL_COUNT)
static union
{
  uint16_t boards[BUS_COUNT] = {};
  uint8_t boardsBytes[BUS_COUNT * 2];
};
#define BOARD_COUNT_SIZE (2)
static union
{
  uint16_t boardCount = 0;
  uint8_t boardCountBytes[BOARD_COUNT_SIZE];
};
#define USB_RLY BOARD_COUNT_MAX
#define USB_RLY_MASK (1 << USB_RLY)

#define COMMAND_SIZE (4)
struct COMMAND
{
  uint8_t bus;
  uint8_t address;
  uint8_t command;
  uint8_t data;
};

static union
{
  COMMAND command = {};
  uint8_t commandBytes[COMMAND_SIZE];
};

#define forEachBoard(stuff)                                    \
  {                                                            \
    uint16_t currentBoard = 0;                                 \
    bool isLastBoard = false;                                  \
    for (uint8_t bus = 0; bus < BUS_COUNT; bus++)              \
    {                                                          \
      for (uint8_t address = 0; address <= USB_RLY; address++) \
      {                                                        \
        if (boards[bus] & (1 << address))                      \
        {                                                      \
          isLastBoard = currentBoard == boardCount - 1;        \
          stuff;                                               \
          currentBoard++;                                      \
        }                                                      \
      }                                                        \
    }                                                          \
  };

/***** Serial event handler *****/
enum BUFFER_STATUS : uint8_t
{
  BS_BUFFERING,
  BS_COMMAND,
  BS_DATA,
};

enum IDN_SEND : uint8_t
{
  ID_DISABLED,
  ID_NAME,
  ID_NAME_SERIAL
};

#define PBSIZE 128
static char pBuf[PBSIZE];
static uint8_t pbPtr = 0;

// CR/LF terminated line ready to process
BUFFER_STATUS lnRdy = BS_BUFFERING;

// Verbose mode
bool isVerb = false;

// Send response to *idn?
IDN_SEND enableIdn = ID_NAME_SERIAL;
bool sendIdn = false;

// Escaped character flag
bool isEsc = false;         // Charcter escaped
bool isPlusEscaped = false; // Plus escaped

/***** Add character to the buffer *****/
inline void addPbuf(char c)
{
  pBuf[pbPtr++] = c;
}

/***** Clear the parse buffer *****/
inline void flushPbuf()
{
  memset(pBuf, '\0', PBSIZE);
  pbPtr = 0;
}

/***** Unrecognized command *****/
void errBadCmd()
{
  Serial.println(F("Unrecognized command"));
}

inline BUFFER_STATUS serialIn_h()
{
  BUFFER_STATUS bufferStatus = BS_BUFFERING;
  // Parse serial input until we have detected a line terminator
  while (Serial.available() && bufferStatus == BS_BUFFERING)
  { // Parse while characters available and line is not complete
    bufferStatus = parseInput(Serial.read());
  }

#ifdef DEBUG1
  if (bufferStatus)
  {
    Serial.print(F("BufferStatus: "));
    Serial.println(bufferStatus);
  }
#endif

  return bufferStatus;
}

/***** Control characters *****/
#define ETX 0x03  // End-of-Text
#define LF 0x0A   // Newline/linefeed
#define CR 0x0D   // Carriage return
#define ESC 0x1B  // the USB escape char
#define PLUS 0x2B // '+' character

//#define DEBUG1

BUFFER_STATUS parseInput(char c)
{
  BUFFER_STATUS r = BS_BUFFERING;

  // Read until buffer full (buffer-size - 2 characters)
  if (pbPtr < PBSIZE)
  {
    if (isVerb)
      Serial.print(c); // Humans like to see what they are typing...
    // Actions on specific characters
    switch (c)
    {
    // Carriage return or newline? Then process the line
    case CR:
    case LF:
      // If escaped just add to buffer
      if (isEsc)
      {
        addPbuf(c);
        isEsc = false;
      }
      else
      {
        // Carriage return on blank line?
        // Note: for data CR and LF will always be escaped
        if (pbPtr == 0)
        {
          flushPbuf();
          if (isVerb)
            showPrompt();
          return BS_BUFFERING;
        }
        else
        {
          if (isVerb)
            Serial.println(); // Move to new line
#ifdef DEBUG1
          Serial.print(F("parseInput: Received "));
          Serial.println(pBuf);
#endif
          // Buffer starts with ++ and contains at least 3 characters - command?
          if (!isPlusEscaped && pbPtr > 2 && isCmd(pBuf))
          {
            r = BS_COMMAND;
          }
          // Buffer contains *idn? query and interface to respond
          else if (enableIdn != ID_DISABLED && pbPtr > 3 && isIdnQuery(pBuf))
          {
            sendIdn = true;
            flushPbuf();
          }
          // Buffer has at least 1 character = instrument data to send to gpib bus
          else if (pbPtr > 0)
          {
            r = BS_DATA;
          }
          isPlusEscaped = false;
#ifdef DEBUG1
          Serial.print(F("R: "));
          Serial.println(r);
#endif
          //            return r;
        }
      }
      break;
    case ESC:
      // Handle the escape character
      if (isEsc)
      {
        // Add character to buffer and cancel escape
        addPbuf(c);
        isEsc = false;
      }
      else
      {
        // Set escape flag
        isEsc = true; // Set escape flag
      }
      break;
    case PLUS:
      if (isEsc)
      {
        if (pbPtr < 2)
          isPlusEscaped = true;
        isEsc = false;
      }
      addPbuf(c);
      //        if (isVerb) arSerial->print(c);
      break;
    // Something else?
    default: // any char other than defined above
      // if (isVerb) arSerial->print(c);  // Humans like to see what they are typing...
      addPbuf(c);
      isEsc = false;
    }
  }

  if (pbPtr >= PBSIZE && r == BS_BUFFERING)
  {
    if (isCmd(pBuf))
    {
      // Command without terminator and buffer full
      if (isVerb)
      {
        Serial.println(F("ERROR - Command buffer overflow!"));
      }
      flushPbuf();
    }
    else
    {
      // Command without terminator and buffer full
      if (isVerb)
      {
        Serial.println(F("ERROR - Control buffer overflow!"));
      }
      flushPbuf();
    }
  }

  return r;
}

/***** Is this a command? *****/
inline bool isCmd(char *buffr)
{
  if (buffr[0] == PLUS && buffr[1] == PLUS)
  {
#ifdef DEBUG1
    Serial.println(F("isCmd: Command detected."));
#endif
    return true;
  }
  return false;
}

/***** Is this an *idn? query? *****/
inline bool isIdnQuery(char *buffr)
{
  // Check for upper or lower case *idn?
  if (strncasecmp(buffr, "*idn?", 5) == 0)
  {
#ifdef DEBUG1
    Serial.println(F("isIdnQuery: Detected IDN query."));
#endif
    return true;
  }
  return false;
}

/***** Show a prompt *****/
inline void showPrompt()
{
  // Print prompt
  Serial.println();
  Serial.print("> ");
}

void setup()
{
  STRING_PRODUCT_PTR=STRING_PRODUCT;
  STRING_PRODUCT_LEN=strlen(HW_NAME);
  STRING_MANUFACTURER_PTR=STRING_MAN;
  STRING_MANUFACTURER_LEN=strlen(HW_MAN);
  STRING_SERIAL_PTR=STRING_SER;
  STRING_SERIAL_LEN=strlen(HW_SER);

  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeout(I2C_TMO);

  Serial.begin(9600);

  availableBoards();
  clearAllRelays();
}

void loop()
{
  //if (Serial.available())
  //{
  //  processCommand(Serial.read());
  //}

  // lnRdy=1: received a command so execute it...
  if (lnRdy == BS_COMMAND)
  {
    execCmd(pBuf, pbPtr);
  }
  else if (lnRdy == BS_DATA)
  {
    execCtrl(pBuf, pbPtr);
  }

  // IDN query ?
  if (sendIdn)
  {
    if (enableIdn == ID_NAME)
    {
      Serial.println(HWID);
    }
    else if (enableIdn == ID_NAME_SERIAL)
    {
      Serial.print(HWID);
      Serial.print("-");
      Serial.println(HWSER);
    }
    sendIdn = false;
  }

  // Check serial buffer
  lnRdy = serialIn_h();

  delayMicroseconds(1);
}

/***** Check whether a parameter is in range *****/
/* Convert string to integer and check whether value is within
   lowl to higl inclusive. Also returns converted text in param
   to a uint16_t integer in rval. Returns true if successful,
   false if not
*/
bool notInRange(char *param, uint16_t lowl, uint16_t higl, uint16_t &rval)
{
  // Null string passed?
  if (strlen(param) == 0)
    return true;

  // Convert to integer
  rval = atoi(param);

  // Check range
  if (rval < lowl || rval > higl)
  {
    errBadCmd();
    if (isVerb)
    {
      Serial.print(F("Valid range is between "));
      Serial.print(lowl);
      Serial.print(F(" and "));
      Serial.println(higl);
    }
    return true;
  }
  return false;
}

/***** Execute a command *****/
void execCmd(char *buffr, uint8_t dsize)
{
  char line[PBSIZE];

  // Copy collected chars to line buffer
  memcpy(line, &buffr[2], dsize - 2);

  // Flush the parse buffer
  flushPbuf();
  lnRdy = BS_BUFFERING;

#ifdef DEBUG1
  Serial.print(F("execCmd: Command received: "));
  Serial.println(line);
#endif

  // Its a ++command so shift everything two bytes left (ignore ++) and parse
  //for (int i = 0; i < dsize - 2; i++)
  //{
  //  line[i] = line[i + 2];
  //}

  // Replace last two bytes with a null (\0) character
  line[dsize - 2] = '\0';
  line[dsize - 1] = '\0';

#ifdef DEBUG1
  Serial.print(F("execCmd: Sent to the command processor: "));
  Serial.println(line);
#endif
  // Execute the command
  if (isVerb)
    Serial.println(); // Shift output to next line
  getCmd(line);

  // Show a prompt on completion?
  if (isVerb)
    showPrompt();
}

/***** Execute a control *****/
void execCtrl(char *buffr, uint8_t dsize)
{
  char line[PBSIZE+2];

  // Copy collected chars to line buffer
  memcpy(line, &buffr[0], dsize);

  // Flush the parse buffer
  flushPbuf();
  lnRdy = BS_BUFFERING;


#ifdef DEBUG1
  Serial.print(F("execCtrl: Command received: "));
  Serial.println(line);
#endif

  line[dsize] = '\0';
  line[dsize+1] = '\0';

#ifdef DEBUG1
  Serial.print(F("execCtrl: Sent to the command processor: "));
  Serial.println(line);
#endif

  // Execute the command
  if (isVerb)
    Serial.println(); // Shift output to next line
  getCtrl(line);

  // Show a prompt on completion?
  if (isVerb)
    showPrompt();
}

/***** Comand function record *****/
struct cmdRec
{
  const char *token;
  void (*handler)(char *);
};

/***** Array containing index of accepted ++ commands *****/
/*
   Commands without parameters require casting to a pointer
   requiring a char* parameter. The functon is called with
   NULL by the command processor.

   Format: token, mode, function_ptr
   Mode: 1=device; 2=controller; 3=both;
*/
static constexpr const cmdRec cmdHidx[] = {
    {"ver", ver_h},
    {"verbose", verb_h},
};

/***** Display the controller version string *****/
void ver_h(char *params)
{
    Serial.print(FWVER);
    Serial.print(", ");
    Serial.println(HWSER);
}

/***** Enable verbose mode 0=OFF; 1=ON *****/
void verb_h(char *params)
{
  if (params != NULL)
  {
    isVerb = false;
  }
  else
  {
    isVerb = !isVerb;
  }
  Serial.print("Verbose: ");
  Serial.println(isVerb ? "ON" : "OFF");
}

void idn_h(char *params)
{
  uint16_t val;
  if (params != NULL)
  {
    if (notInRange(params, 0, 2, val))
      return;
    enableIdn = (IDN_SEND)val;
    if (isVerb)
    {
      Serial.print(F("Sending IDN: "));
      Serial.print(val ? "Enabled" : "Disabled");
      if (val == 2)
        Serial.print(F(" with serial number"));
      Serial.println();
    };
  }
  else
  {
    Serial.println(enableIdn, DEC);
  }
}

static constexpr const cmdRec cmdCtrl[] = {
    {"bus", bus_c},
    {"addr", addr_c},
    {"data", data_c},
    {"cmd", cmd_c},
    {"run", run_c},
    {"clearAll", (void (*)(char *))clearAllRelays},
    {"setAll", (void (*)(char *))setAllRelays},
    {"getAll", (void (*)(char *))getAllRelays},
    {"set", (void (*)(char *))set_c},
    {"get", (void (*)(char *))get_c},
    {"boards", (void (*)(char *))getBoards},
    {"scan", (void (*)(char *))scanBoards},
    {"check", (void (*)(char *))checkBoards},
};

void bus_c(char *params)
{
  uint16_t val;
  if (params != NULL)
  {
    if (notInRange(params, 0, BUS_COUNT - 1, val))
      return;
    command.bus = val;
    if (isVerb)
    {
      Serial.print(F("Set bus to: "));
      Serial.println(command.bus);
    }
  }
  else
  {
    Serial.println(command.bus);
  }
}

void addr_c(char *params)
{
  uint16_t val;
  if (params != NULL)
  {
    if (notInRange(params, 0, BOARD_COUNT_MAX, val))
      return;
    command.address = val;
    if (isVerb)
    {
      Serial.print(F("Set address to: "));
      Serial.println(command.address);
    }
  }
  else
  {
    Serial.println(command.address);
  }
}

void data_c(char *params)
{
  uint16_t val;
  if (params != NULL)
  {
    if (notInRange(params, 0x00, 0xFF, val))
      return;
    command.data = val;
    if (isVerb)
    {
      Serial.print(F("Set data to: "));
      Serial.println(command.data);
    }
  }
  else
  {
    Serial.println(command.data);
  }
}

void cmd_c(char *params)
{
  uint16_t val;
  if (params != NULL)
  {
    if (notInRange(params, 0x00, 0xFF, val))
      return;
    command.command = val;
    if (isVerb)
    {
      Serial.print(F("Set command to: "));
      Serial.println(command.command);
    }
  }
  else
  {
    Serial.println(command.command);
  }
}

void run_c(char *params)
{
  char *param;
  uint8_t cnt = 0;
  uint16_t val = 0;

  if (params != NULL)
  {
    // Read address parameters into array
    while (cnt < 4)
    {
      if (cnt == 0)
      {
        param = strtok(params, " ,");
      }
      else
      {
        param = strtok(NULL, " ,");
      }

      switch (cnt)
      {
      case 0:
        if (notInRange(param, 0, BUS_COUNT - 1, val))
          return;
        command.bus = val;
        break;
      case 1:
        if (notInRange(param, 0, BOARD_COUNT_MAX, val))
          return;
        command.address = val;
        break;
      case 2:
        if (notInRange(param, 0x00, 0xFF, val))
          return;
        command.command = val;
        break;
      case 3:
        if (notInRange(param, 0x00, 0xFF, val))
          return;
        command.data = val;
        break;
      }
      cnt++;
    }
  }

  runCommand(command);
  Serial.println();
}

void set_c(char *params)
{
  if (boardCount < 1)
  {
    return;
  }

  char *param;
  uint8_t cnt = 0;
  uint16_t val = 0;

  uint16_t offset;
  uint8_t states[32];

  if (params != NULL)
  {
    param = strtok(params, " ,");
    if (notInRange(param, 0, boardCount - 1, val))
      return;
    offset = val;
    // Read address parameters into array
    while (cnt < 32)
    {
      param = strtok(NULL, " ,");
      if (notInRange(param, 0x00, 0xFF, val))
        break;
      states[cnt] = (uint8_t)val;
      cnt++;
    }

    if (cnt > 0)
    {
      setRelays(offset, cnt, states);
    }
  }
}

void get_c(char *params)
{
  if (boardCount < 1)
  {
    return;
  }

  char *param;
  uint8_t cnt = 0;
  uint16_t val = 0;

  uint16_t offset;

  if (params != NULL)
  {
    // Read address parameters into array
    while (cnt < 2)
    {
      switch (cnt)
      {
      case 0:
        param = strtok(params, " ,");
        if (notInRange(param, 0, boardCount - 1, val))
          return;
        offset = val;
        val = boardCount - offset;
        break;
      default:
        param = strtok(NULL, " ,");
        if (notInRange(param, 1, boardCount - offset, val))
          return;
        break;
      }
      cnt++;
    }

    getRelays(offset, val);
  }
}

/***** Extract command and pass to handler *****/
void getCmd(char *buffr)
{
  char *token;  // Pointer to command token
  char *params; // Pointer to parameters (remaining buffer characters)

  int casize = sizeof(cmdHidx) / sizeof(cmdHidx[0]);
  int i = 0;

#ifdef DEBUG1
  Serial.print("getCmd: ");
  Serial.print(buffr);
  Serial.print(F(" - length:"));
  Serial.println(strlen(buffr));
#endif

  // If terminator on blank line then return immediately without processing anything
  if (buffr[0] == 0x00 || buffr[0] == CR || buffr[0] == LF)
    return;

  // Get the first token
  token = strtok(buffr, " \t");

#ifdef DEBUG1
  Serial.print("getCmd: process token: ");
  Serial.println(token);
#endif

  do
  {
    // Check whether it is a valid command token
    if (strcasecmp(cmdHidx[i].token, token) == 0)
      break;
    i++;
  } while (i < casize);

  if (i < casize)
  {
    // We have found a valid command and handler
#ifdef DEBUG1
    Serial.print("getCmd: found handler for: ");
    Serial.println(cmdHidx[i].token);
#endif
    // If command is relevant to mode then execute it

    // If its a command with parameters
    // Copy command parameters to params and call handler with parameters
    params = token + strlen(token) + 1;

    // If command parameters were specified
    if (strlen(params) > 0)
    {
#ifdef DEBUG1
      Serial.print(F("Calling handler with parameters: "));
      Serial.println(params);
#endif
      // Call handler with parameters specified
      cmdHidx[i].handler(params);
    }
    else
    {
      // Call handler without parameters
      cmdHidx[i].handler(NULL);
    }
  }
  else
  {
    // No valid command found
    errBadCmd();
  }
}

/***** Extract control and pass to handler *****/
void getCtrl(char *buffr)
{
  char *token;  // Pointer to command token
  char *params; // Pointer to parameters (remaining buffer characters)

  int casize = sizeof(cmdCtrl) / sizeof(cmdCtrl[0]);
  int i = 0;

#ifdef DEBUG1
  Serial.print("getCtrl: ");
  Serial.print(buffr);
  Serial.print(F(" - length:"));
  Serial.println(strlen(buffr));
#endif

  // If terminator on blank line then return immediately without processing anything
  if (buffr[0] == 0x00 || buffr[0] == CR || buffr[0] == LF)
    return;

  // Get the first token
  token = strtok(buffr, " \t");

#ifdef DEBUG1
  Serial.print("getCtrl: process token: ");
  Serial.println(token);
#endif

  do
  {
    // Check whether it is a valid command token
    if (strcasecmp(cmdCtrl[i].token, token) == 0)
      break;
    i++;
  } while (i < casize);

  if (i < casize)
  {
    // We have found a valid command and handler
#ifdef DEBUG1
    Serial.print("getCtrl: found handler for: ");
    Serial.println(cmdCtrl[i].token);
#endif
    // If command is relevant to mode then execute it

    // If its a command with parameters
    // Copy command parameters to params and call handler with parameters
    params = token + strlen(token) + 1;

    // If command parameters were specified
    if (strlen(params) > 0)
    {
#ifdef DEBUG1
      Serial.print(F("Calling handler with parameters: "));
      Serial.println(params);
#endif
      // Call handler with parameters specified
      cmdCtrl[i].handler(params);
    }
    else
    {
      // Call handler without parameters
      cmdCtrl[i].handler(NULL);
    }
  }
  else
  {
    // No valid command found
    errBadCmd();
  }
}

/*
inline void processCommand(uint8_t cmdPart)
{
  uint8_t cmdType = cmdPart >> 4;
  uint8_t cmdData = cmdPart & 0x0F;

  if (cmdType < COMMAND_SIZE * 2)
  {
    if (cmdType & 1)
    {
      commandBytes[cmdType >> 1] &= 0x0F;
      commandBytes[cmdType >> 1] |= cmdData << 4;
    }
    else
    {
      commandBytes[cmdType >> 1] &= 0xF0;
      commandBytes[cmdType >> 1] |= cmdData;
    }
  }
  else if (cmdType == 0xF)
  {
    switch (cmdData)
    {
    case 0:
      clearAllRelays();
      break;
    case 1:
      setAllRelays();
      break;
    case 2:
      getAllRelays();
      break;
    case 3:
      getBoards();
      break;
    case 4:
      scanBoards();
      break;
      
    case 0xC:
      Serial.write(commandBytes, COMMAND_SIZE);
      break;
    case 0xD:
      command = {};
      break;
    case 0xE:
      runCommand(command);
      break;
    case 0xF:
      runCommand(command);
      command = {};
      break;
    }
  }
}
*/

#pragma region available boards

void availableBoards()
{
  digitalWrite(LED_BUILTIN, LOW);
#ifdef USE_SERIAL_STATE
  if (Serial)
  {
    sendSerialState(0);
  }
#endif
  for (uint8_t i = 0; i < BUS_COUNT; i++)
  {
    boards[i] = 0;
  }
  boardCount = 0;
  boardCount += availableBoardsHardwareI2C();
  boardCount += availableBoardsSoftwareI2C();
  boardCount += availableBoardsHardwareSerial();
  boardCount += availableBoardsSoftwareSerial();
  digitalWrite(LED_BUILTIN, HIGH);
#ifdef USE_SERIAL_STATE
  if (Serial)
  {
    sendSerialState(SS_DSR | SS_DCD);
  }
#endif
}

uint8_t availableBoardsHardwareI2C()
{
  uint8_t count = 0;
  uint16_t list = 0;
  for (uint8_t address = BOARD_ADDR_MIN; address < BOARD_ADDR_MIN + BOARD_COUNT_MAX; address++)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
    {
      list |= 1 << (address - BOARD_ADDR_MIN);
      count++;
    }
  }
  boards[I2C_BUS] = list;
  return count;
}

uint8_t availableBoardsSoftwareI2C()
{
  return 0;
}

uint8_t availableBoardsHardwareSerial()
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < HARDWARE_SERIAL_COUNT; i++)
  {
    uint16_t list = 0;
    uint8_t id[2] = {};

#define serial hardwareSerials[i]
    serial->begin(19200); //check for "USB" rly
    serial->setTimeout(USB_RLY_TMO);

    serial->write(90);
    if (2 == serial->readBytes(id, 2) && (id[0] == 8 || id[0] == 9))
    {
      list |= USB_RLY_MASK;
      count++;
    }
    serial->end();

    if (count == 0)
    {
      serial->begin(9600);
      serial->setTimeout(RLY_TMO);

      for (uint8_t addr = 0; addr < BOARD_COUNT_MAX; addr++)
      {
        serial->write(addr + 1);
        serial->write(90);
        if (serial->readBytes(id, 1))
        {
          list |= 1 << addr;
          count++;
        }
      }
      serial->end();
    }

#undef serial

    boards[HARDWARE_SERIAL_BUS + i] = list;
  }
  return count;
}

uint8_t availableBoardsSoftwareSerial()
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < SOFTWARE_SERIAL_COUNT; i++)
  {
    uint16_t list = 0;
    uint8_t id[2] = {};

#define serial softwareSerials[i]
    serial->begin(19200); //check for "USB" rly
    serial->setTimeout(USB_RLY_TMO);

    serial->write(90);
    if (2 == serial->readBytes(id, 2) && (id[0] == 8 || id[0] == 9))
    {
      list |= USB_RLY_MASK;
      count++;
    }
    serial->end();

    if (count == 0)
    {
      serial->begin(9600);
      serial->setTimeout(RLY_TMO);

      for (uint8_t addr = 0; addr < BOARD_COUNT_MAX; addr++)
      {
        serial->write(addr + 1);
        serial->write(90);
        if (serial->readBytes(id, 1))
        {
          list |= 1 << addr;
          count++;
        }
      }
      serial->end();
    }

#undef serial

    boards[SOFTWARE_SERIAL_BUS + i] = list;
  }
  return count;
}

#pragma endregion

inline uint16_t runCommand(COMMAND command)
{
  return runCommand(command.bus, command.address, command.command, command.data);
}

uint16_t runCommand(uint8_t bus, uint8_t address, uint8_t command, uint8_t data)
{
  union
  {
    uint16_t ret = 0;
    uint8_t id[2];
  };

  if (bus >= I2C_BUS && bus < I2C_BUS + I2C_COUNT)
  {
    bus -= I2C_BUS;
    if (address < 8)
    {
      address += BOARD_ADDR_MIN;
    }

    switch (command)
    {
    case 90:
      Wire.beginTransmission(address);
      Wire.write(CMD_REGISTER);
      Wire.endTransmission();
      Wire.requestFrom(address, (uint8_t)1);
      id[1] = Wire.read();
      Wire.endTransmission();
      break;
    case 91:
      Wire.beginTransmission(address);
      Wire.write(RLY_REGISTER);
      Wire.endTransmission();
      Wire.requestFrom(address, (uint8_t)1);
      id[1] = Wire.read();
      Wire.endTransmission();
      break;
    case 92:
      Wire.beginTransmission(address);
      Wire.write(RLY_REGISTER);
      Wire.write(data);
      Wire.endTransmission();
      break;
    default:
      Wire.beginTransmission(address);
      Wire.write(CMD_REGISTER);
      Wire.write(command);
      Wire.endTransmission();
      break;
    }
  }
  else if (bus >= SOFTWARE_I2C_BUS && bus < SOFTWARE_I2C_BUS + SOFTWARE_I2C_COUNT)
  {
    bus -= SOFTWARE_I2C_BUS;
    if (address < BOARD_COUNT_MAX)
    {
      address += BOARD_ADDR_MIN;
    }
    //TODO
  }
  else if (bus >= HARDWARE_SERIAL_BUS && bus < HARDWARE_SERIAL_BUS + HARDWARE_SERIAL_COUNT)
  {
    bus -= HARDWARE_SERIAL_BUS;
#define serial hardwareSerials[bus]
    if (address == USB_RLY)
    {
      serial->begin(19200); //check for "USB" rly
      serial->setTimeout(USB_RLY_TMO);
    }
    else
    {
      serial->begin(9600);
      serial->setTimeout(RLY_TMO);
      serial->write(address + 1);
    }
    serial->write(command);

    switch (command)
    {
    case 90:
      if (address == USB_RLY)
      {
        serial->readBytes(id + 1, 1);
      }
      else
      {
        serial->readBytes(id, 2);
      }
      break;
    case 91:
      serial->readBytes(id + 1, 1);
      break;
    case 92:
      serial->write(data);
      break;
    case 93:
      if (address == USB_RLY)
      {
        serial->readBytes(id + 1, 1);
      }
      else
      {
        id[1] = 50;
      }
      break;
    default:
      break;
    }

    serial->end();
#undef serial
  }
  else if (bus >= SOFTWARE_SERIAL_BUS && bus < SOFTWARE_SERIAL_BUS + SOFTWARE_SERIAL_COUNT)
  {
    bus -= SOFTWARE_SERIAL_BUS;
#define serial softwareSerials[bus]
    if (address == USB_RLY)
    {
      serial->begin(19200); //check for "USB" rly
      serial->setTimeout(USB_RLY_TMO);
    }
    else
    {
      serial->begin(9600);
      serial->setTimeout(RLY_TMO);
      serial->write(address + 1);
    }
    serial->write(command);

    switch (command)
    {
    case 90:
      if (address == USB_RLY)
      {
        serial->readBytes(id + 1, 1);
      }
      else
      {
        serial->readBytes(id, 2);
      }
      break;
    case 91:
      serial->readBytes(id + 1, 1);
      break;
    case 92:
      serial->write(data);
      break;
    case 93:
      if (address == USB_RLY)
      {
        serial->readBytes(id + 1, 1);
      }
      else
      {
        id[1] = 50;
      }
      break;
    default:
      break;
    }

    serial->end();
#undef serial
  }

  Serial.print(id[0]); // Send the upper byte first
  Serial.print(" ,");
  Serial.print(id[1]);
  return ret;
}

#pragma region macro commands

void clearAllRelays()
{
  forEachBoard(
      runCommand(bus, address, 110, 0);
      if (!isLastBoard) {
        Serial.print(" ,");
      });
  Serial.println();
}

void setAllRelays()
{
  forEachBoard(
      runCommand(bus, address, 100, 0);
      if (!isLastBoard) {
        Serial.print(" ,");
      });
  Serial.println();
}

void getAllRelays()
{
  forEachBoard(
      runCommand(bus, address, 91, 0);
      if (!isLastBoard) {
        Serial.print(" ,");
      });
  Serial.println();
}

void setRelays(uint16_t offset, uint8_t size, uint8_t *states)
{
  forEachBoard(
      if (currentBoard >= offset)
      {
        runCommand(bus, address, 92, states[currentBoard - offset]);
        size--;
        if (!isLastBoard && size > 0)
        {
          Serial.print(" ,");
        }
        else
        {
          goto done;
        }
      });
done:
  Serial.println();
}

void getRelays(uint16_t offset, uint8_t size)
{
  forEachBoard(
      if (currentBoard >= offset)
      {
        runCommand(bus, address, 91, 0);
        size--;
        if (!isLastBoard && size > 0)
        {
          Serial.print(" ,");
        }
        else
        {
          goto done;
        }
      });
done:
  Serial.println();
}

void getBoards()
{
  Serial.print(boardCount); // Send the upper byte first
  if (boardCount > 0)
    Serial.print(" ,");
  forEachBoard(
      Serial.print(bus);
      Serial.print(" ,");
      Serial.print(address);
      if (!isLastBoard)
      {
        Serial.print(" ,");
      });
  Serial.println();
}

void scanBoards()
{
  availableBoards();
  getBoards();
}

void checkBoards()
{
  Serial.print(boardCount); // Send the upper byte first
  if (boardCount > 0)
    Serial.print(" ,");
  forEachBoard(
      runCommand(bus, address, 90, 0);
      if (!isLastBoard) {
        Serial.print(" ,");
      });
  Serial.println();
}

#pragma endregion
