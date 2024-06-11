#define SERIAL_4WAY_INTERFACE_NAME_STR "m4wFCIntf"
// *** change to adapt Revision
#define SERIAL_4WAY_VER_MAIN 20
#define SERIAL_4WAY_VER_SUB_1 (uint8_t)0
#define SERIAL_4WAY_VER_SUB_2 (uint8_t)05

#define SERIAL_4WAY_PROTOCOL_VER 108

#define SERIAL_4WAY_VERSION (uint16_t)((SERIAL_4WAY_VER_MAIN * 1000) + (SERIAL_4WAY_VER_SUB_1 * 100) + SERIAL_4WAY_VER_SUB_2)

#define SERIAL_4WAY_VERSION_HI (uint8_t)(SERIAL_4WAY_VERSION / 100)
#define SERIAL_4WAY_VERSION_LO (uint8_t)(SERIAL_4WAY_VERSION % 100)

#define imARM_BLB 4

// Send Structure
// ESC + CMD PARAM_LEN [PARAM (if len > 0)] CRC16_Hi CRC16_Lo
// Return
// ESC CMD PARAM_LEN [PARAM (if len > 0)] + ACK (uint8_t OK or ERR) + CRC16_Hi CRC16_Lo

#define cmd_Remote_Escape 0x2E // '.'
#define cmd_Local_Escape 0x2F  // '/'

// Test Interface still present
#define cmd_InterfaceTestAlive 0x30 // '0' alive
// RETURN: ACK

// get Protocol Version Number 01..255
#define cmd_ProtocolGetVersion 0x31 // '1' version
// RETURN: uint8_t VersionNumber + ACK

// get Version String
#define cmd_InterfaceGetName 0x32 // '2' name
// RETURN: String + ACK

// get Version Number 01..255
#define cmd_InterfaceGetVersion 0x33 // '3' version
// RETURN: uint8_t AVersionNumber + ACK

// Exit / Restart Interface - can be used to switch to Box Mode
#define cmd_InterfaceExit 0x34 // '4' exit
// RETURN: ACK

// Reset the Device connected to the Interface
#define cmd_DeviceReset 0x35 // '5' reset
// RETURN: ACK

// Initialize Flash Access for Device connected
#define cmd_DeviceInitFlash 0x37 // '7' init flash access
// RETURN: ACK

// Erase the whole Device Memory of connected Device
#define cmd_DeviceEraseAll 0x38 // '8' erase all
// RETURN: ACK

// Erase one Page of Device Memory of connected Device
#define cmd_DevicePageErase 0x39 // '9' page erase
// PARAM: uint8_t APageNumber
// RETURN: ACK

// Read to Buffer from Device Memory of connected Device // Buffer Len is Max 256 Bytes
// BuffLen = 0 means 256 Bytes
#define cmd_DeviceRead 0x3A // ':' read Device
// PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BuffLen[0..255]
// RETURN: PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255] ACK

// Write to Buffer for Device Memory of connected Device // Buffer Len is Max 256 Bytes
// BuffLen = 0 means 256 Bytes
#define cmd_DeviceWrite 0x3B // ';' write
// PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255]
// RETURN: ACK

// Set C2CK low infinite ) permanent Reset state
#define cmd_DeviceC2CK_LOW 0x3C // '<'
// RETURN: ACK

// Read to Buffer from Device Memory of connected Device //Buffer Len is Max 256 Bytes
// BuffLen = 0 means 256 Bytes
#define cmd_DeviceReadEEprom 0x3D // '=' read Device
// PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BuffLen[0..255]
// RETURN: PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255] ACK

// Write to Buffer for Device Memory of connected Device // Buffer Len is Max 256 Bytes
// BuffLen = 0 means 256 Bytes
#define cmd_DeviceWriteEEprom 0x3E // '>' write
// PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255]
// RETURN: ACK

// Set Interface Mode
#define cmd_InterfaceSetMode 0x3F // '?'
// PARAM: uint8_t Mode
// RETURN: ACK or ACK_I_INVALID_CHANNEL

// Write to Buffer for Verify Device Memory of connected Device //Buffer Len is Max 256 Bytes
// BuffLen = 0 means 256 Bytes
#define cmd_DeviceVerify 0x40 //'@' write
// PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255]
// RETURN: ACK

// responses
#define ACK_OK 0x00
// #define ACK_I_UNKNOWN_ERROR       0x01
#define ACK_I_INVALID_CMD 0x02
#define ACK_I_INVALID_CRC 0x03
#define ACK_I_VERIFY_ERROR 0x04
// #define ACK_D_INVALID_COMMAND 0x05
// #define ACK_D_COMMAND_FAILED  0x06
// #define ACK_D_UNKNOWN_ERROR       0x07

#define ACK_I_INVALID_CHANNEL 0x08
#define ACK_I_INVALID_PARAM 0x09
#define ACK_D_GENERAL_ERROR 0x0F

// Bootloader commands
// RunCmd
#define RestartBootloader 0
#define ExitBootloader 1

#define CMD_RUN 0x00
#define CMD_PROG_FLASH 0x01
#define CMD_ERASE_FLASH 0x02
#define CMD_READ_FLASH_SIL 0x03
#define CMD_VERIFY_FLASH 0x03
#define CMD_VERIFY_FLASH_ARM 0x04
#define CMD_READ_EEPROM 0x04
#define CMD_PROG_EEPROM 0x05
#define CMD_READ_SRAM 0x06
#define CMD_READ_FLASH_ATM 0x07
#define CMD_KEEP_ALIVE 0xFD
#define CMD_SET_ADDRESS 0xFF
#define CMD_SET_BUFFER 0xFE

#define CMD_BOOTINIT 0x07
#define CMD_BOOTSIGN 0x08

// Bootloader result codes
#define brSUCCESS 0x30
#define brERRORVERIFY 0xC0
#define brERRORCOMMAND 0xC1
#define brERRORCRC 0xC2
#define brNONE 0xFF

uint16_t Check_4Way(uint8_t buf[]);
uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data);

extern uint8_t passthroughBreakout;