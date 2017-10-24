/**
 * @file modpoll.cpp
 *
 * @if NOTICE
 *
 * Copyright (c) proconX Pty Ltd. All rights reserved.
 *
 * The following source file constitutes example program code and is
 * intended merely to illustrate useful programming techniques.  The user
 * is responsible for applying the code correctly.
 *
 * THIS SOFTWARE IS PROVIDED BY PROCONX AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL PROCONX OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @endif
 */


// Platform header
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <float.h>
#ifdef _WIN32
#  include "getopt.h"
#else
#  include <unistd.h>
#endif

// Include FieldTalk package header
#include "MbusRtuMasterProtocol.hpp"
#include "MbusAsciiMasterProtocol.hpp"
#include "MbusTcpMasterProtocol.hpp"
#include "MbusRtuOverTcpMasterProtocol.hpp"


/*****************************************************************************
 * String constants
 *****************************************************************************/

const char versionStr[]= "3.4";
const char progName[] = "modpoll";
const char bannerStr[] =
"%s %s - FieldTalk(tm) Modbus(R) Master Simulator\n"
"Copyright (c) 2002-2013 proconX Pty Ltd\n"
"Visit http://www.modbusdriver.com for Modbus libraries and tools.\n"
"\n";

const char usageStr[] =
"%s [OPTIONS] SERIALPORT|HOST [WRITEVALUES...]\n"
"Arguments: \n"
"SERIALPORT    Serial port when using Modbus ASCII or Modbus RTU protocol \n"
"              COM1, COM2 ...                on Windows \n"
"              /dev/ttyS0, /dev/ttyS1 ...    on Linux \n"
"              /dev/ser1, /dev/ser2 ...      on QNX \n"
"HOST          Host name or dotted IP address when using MDBUS/TCP protocol\n"
"WRITEVALUES   List of values to be written. If none specified (default) modpoll reads data.\n"
"General options: \n"
"-m ascii      Modbus ASCII protocol\n"
#ifdef __WIN32__
"-m rtu        Modbus RTU protocol (default if SERIALPORT contains \\ or COM)\n"
#else
"-m rtu        Modbus RTU protocol (default if SERIALPORT contains a /)\n"
#endif
"-m tcp        MODBUS/TCP protocol (default otherwise)\n"
"-m enc        Encapsulated Modbus RTU over TCP\n"
"-a #          Slave address (1-255 for serial, 0-255 for TCP, 1 is default)\n"
"-r #          Start reference (1-65536, 100 is default)\n"
"-c #          Number of values to read (1-125, 1 is default)\n"
"-t 0          Discrete output (coil) data type\n"
"-t 1          Discrete input data type\n"
"-t 3          16-bit input register data type\n"
"-t 3:hex      16-bit input register data type with hex display\n"
"-t 3:int      32-bit integer data type in input register table\n"
"-t 3:mod      32-bit module 10000 data type in input register table\n"
"-t 3:float    32-bit float data type in input register table\n"
"-t 4          16-bit output (holding) register data type (default)\n"
"-t 4:hex      16-bit output (holding) register data type with hex display\n"
"-t 4:int      32-bit integer data type in output (holding) register table\n"
"-t 4:mod      32-bit module 10000 type in output (holding) register table\n"
"-t 4:float    32-bit float data type in output (holding) register table\n"
"-i            Slave operates on big-endian 32-bit integers\n"
"-f            Slave operates on big-endian 32-bit floats\n"
"-e            Use Daniel/Enron single register 32-bit mode\n"
"-0            First reference is 0 (PDU addressing) instead 1\n"
"-1            Poll only once only, otherwise every poll rate interval\n"
"-l #          Poll rate in ms, (1000 is default)\n"
"-o #          Time-out in seconds (0.01 - 10.0, 1.0 s is default)\n"
"Options for MODBUS/TCP:\n"
"-p #          TCP port number (502 is default)\n"
"Options for Modbus ASCII and Modbus RTU:\n"
"-b #          Baudrate (e.g. 9600, 19200, ...) (19200 is default)\n"
"-d #          Databits (7 or 8 for ASCII protocol, 8 for RTU)\n"
"-s #          Stopbits (1 or 2, 1 is default)\n"
"-p none       No parity\n"
"-p even       Even parity (default)\n"
"-p odd        Odd parity\n"
"-4 #          RS-485 mode, RTS on while transmitting and another # ms after\n"
"";


/*****************************************************************************
 * Definitions
 *****************************************************************************/

const int MAX_REF_CNT = 125;

enum
{
   RTU,       ///< Modbus RTU protocol
   ASCII,     ///< Modbus ASCII protocol
   TCP,       ///< MODBUS/TCP protocol
   RTUOVERTCP ///< Encapsulated RTU over TCP
};

enum
{
   T0_BIT,     ///< bit data types, discrete output table
   T1_BIT,     ///< bit data types, discrete input table
   T3_REG16,    ///< 16-bit register (short) types, input table
   T3_HEX16,    ///< 16-bit register types with hex display, input table
   T3_INT32,    ///< 32-bit integer types (2 16-bit registers), input table
   T3_MOD10000, ///< 32-bit modulo 10000 types, input table
   T3_FLOAT32,  ///< Float types (2 16-bit registers), input table
   T4_REG16,    ///< 16-bit register (short) types, output table
   T4_HEX16,    ///< 16-bit register types with hex display, output table
   T4_INT32,    ///< 32-bit integer types (2 16-bit registers), output table
   T4_MOD10000, ///< 32-bit modulo 10000 types, output table
   T4_FLOAT32   ///< Float types (2 16-bit registers), output table
};


/*****************************************************************************
 * Gobal configuration data
 *****************************************************************************/

int slaveAddr = 1;
int ref = 1;
int refCnt = -1;
int writeCnt = 0;
int pollCnt = -1;
long baudRate = 19200;
int dataBits = 8;
int stopBits = 1;
int parity = MbusSerialMasterProtocol::SER_PARITY_EVEN;
int protocol = -1;
int dataType = T4_REG16;
int swapInts = 0;
int swapFloats = 0;
int enronMode = 0;
int pduAddressing = 0;
char *portName = NULL;
int port = 502;
int rs485Mode = 0;
int timeOut = 1000;
int pollDelay = 1000;


/*****************************************************************************
 * Protocol and register buffers
 *****************************************************************************/

MbusMasterFunctions *mbusPtr;

short shortArr[MAX_REF_CNT];
int intArr[MAX_REF_CNT * 16]; // Also holds discretes/coils
float floatArr[MAX_REF_CNT];


/*****************************************************************************
 * Function implementation
 *****************************************************************************/

/**
 * Prints a usage message on stdout and exits
 */
void printUsage()
{
   printf(bannerStr, progName, versionStr);
   printf("Usage: ");
   printf(usageStr, progName);
   exit(EXIT_FAILURE);
}


/**
 * Prints version info on stdout
 */
void printVersion()
{
   printf(bannerStr, progName, versionStr);
}


/**
 * Prints the current configuration on stdout
 */
void printConfig()
{
   printf(bannerStr, progName, versionStr);
   printf("Protocol configuration: ");
   switch (protocol)
   {
      case RTU:
         printf("Modbus RTU\n");
      break;
      case ASCII:
         printf("Modbus ASCII\n");
      break;
      case TCP:
         printf("MODBUS/TCP\n");
      break;
      case RTUOVERTCP:
         printf("Encapsulated RTU over TCP\n");
      break;
      default:
         printf("unknown\n");
      break;
   }
   printf("Slave configuration...: ");
   printf("address = %d, ", slaveAddr);
   if (pduAddressing)
      printf("start reference = %d (PDU), ", ref);
   else
      printf("start reference = %d, ", ref);
   printf("count = %d\n", refCnt);
   printf("Communication.........: ");
   if ((protocol == TCP) || (protocol == RTUOVERTCP))
      printf("%s, port %d", portName, port);
   else
   {
      printf("%s, %ld, %d, %d, ", portName,baudRate, dataBits, stopBits);
      switch (parity)
      {
         case MbusSerialMasterProtocol::SER_PARITY_NONE:
            printf("none");
         break;
         case MbusSerialMasterProtocol::SER_PARITY_EVEN:
            printf("even");
         break;
         case MbusSerialMasterProtocol::SER_PARITY_ODD:
            printf("odd");
         break;
         default:
            printf("unknown");
         break;
      }
   }
   printf(", t/o %.2f s", ((float) timeOut) / 1000.0F);
   printf(", poll rate %d ms\n", pollDelay);
   printf("Data type.............: ");
   switch (dataType)
   {
      case T0_BIT:
         printf("discrete output (coil)\n");
      break;
      case T1_BIT:
         printf("discrete input\n");
      break;
      case T3_REG16:
         printf("16-bit register, input register table\n");
      break;
      case T3_HEX16:
         printf("16-bit register (hex), input register table\n");
      break;
      case T3_INT32:
         printf("32-bit integer, input register table\n");
      break;
      case T3_MOD10000:
         printf("32-bit module 10000, input register table\n");
      break;
      case T3_FLOAT32:
         printf("32-bit float, input register table\n");
      break;
      case T4_REG16:
         printf("16-bit register, output (holding) register table\n");
      break;
      case T4_HEX16:
         printf("16-bit register (hex), output (holding) register table\n");
      break;
      case T4_INT32:
         printf("32-bit integer, output (holding) register table\n");
      break;
      case T4_MOD10000:
         printf("32-bit module 10000, output (holding) register table\n");
      break;
      case T4_FLOAT32:
         printf("32-bit float, output (holding) register table\n");
      break;
      default:
         printf("unknown\n");
      break;
   }
   if (swapInts || swapFloats)
   {
      printf("Word swapping.........: Slave configured as big-endian");
      if (swapInts)
         printf(" word");
      if (swapInts && swapFloats)
         printf(" and");
      if (swapFloats)
         printf(" float");
      printf(" machine\n");
   }
   if (enronMode)
      printf("32-bit mode...........: Daniel/Enron single 32-bit word\n");
   printf("\n");
}


/**
 * Prints bad option error message and exits program
 *
 * @param text Option error message
 */
void exitBadOption(const char *const text)
{
   fprintf(stderr, "%s: %s! Try -h for help.\n", progName, text);
   exit(EXIT_FAILURE);
}


/**
 * Scans and parses the command line options.
 *
 * @param argc Argument count
 * @param argv Argument value string array
 */
void scanOptions(int argc, char **argv)
{
   int i;
   int c;

   // Check for --version option
   for (c = 1; c < argc; c++)
   {
      if (strcmp (argv[c], "--version") == 0)
      {
         printVersion();
         exit(EXIT_FAILURE);
      }
   }

   // Check for --help option
   for (c = 1; c < argc; c++)
   {
      if (strcmp (argv[c], "--help") == 0)
         printUsage();
   }

   opterr = 0; // Disable getopt's error messages
   for (;;)
   {
      c = getopt(argc, argv, "h14:if0ea:r:c:b:d:s:p:t:m:o:l:");
      if (c == -1)
         break;

      switch (c)
      {
         case '1':
            pollCnt = 1;
         break;
         case '4':
            rs485Mode = (int) strtol(optarg, NULL, 0);
            if ((rs485Mode <= 0) || (rs485Mode > 1000))
               exitBadOption("Invalid RTS delay parameter");
         break;
         case 'o':
            timeOut = (int) (strtod(optarg, NULL) * 1000.0);
            if ((timeOut < 10) || (timeOut > 10000))
               exitBadOption("Invalid time-out parameter");
         break;
         case 'i':
            swapInts = 1;
         break;
         case 'f':
            swapFloats = 1;
         break;
         case 'e':
            enronMode = 1;
         break;
         case '0':
            pduAddressing = 1;
         break;
         case 'm':
            if (strcmp(optarg, "tcp") == 0)
               protocol = TCP;
            else if (strcmp(optarg, "rtu") == 0)
                  protocol = RTU;
            else if (strcmp(optarg, "ascii") == 0)
               protocol = ASCII;
            else if (strcmp(optarg, "enc") == 0)
               protocol = RTUOVERTCP;
            else
               exitBadOption("Invalid protocol parameter");
         break;
         case 'a':
            slaveAddr = strtol(optarg, NULL, 0);
            if ((slaveAddr < 0) || (slaveAddr > 255))
               exitBadOption("Invalid slave address parameter");
         break;
         case 'r':
            ref = strtol(optarg, NULL, 0);
             if ((ref <= 0) || (ref > 0x10000))
               exitBadOption("Invalid reference parameter");
         break;
         case 'c':
            refCnt = strtol(optarg, NULL, 0);
            // Checked further down as we first have to know the data type
         break;
         case 'b':
            baudRate = strtol(optarg, NULL, 0);
            if (baudRate == 0)
               exitBadOption("Invalid baudrate parameter");
         break;
         case 'd':
            dataBits = (int) strtol(optarg, NULL, 0);
            if ((dataBits != 7) && (dataBits != 8))
               exitBadOption("Invalid databits parameter");
         break;
         case 's':
            stopBits = (int) strtol(optarg, NULL, 0);
            if ((stopBits != 1) && (stopBits != 2))
               exitBadOption("Invalid stopbits parameter");
         break;
         case 'l':
            pollDelay = strtol(optarg, NULL, 0);
            if ((pollDelay > 10000) || (pollDelay < 0))
               exitBadOption("Invalid poll rate parameter");
         break;
         case 'p':
            if (strcmp(optarg, "none") == 0)
               parity = MbusSerialMasterProtocol::SER_PARITY_NONE;
            else if (strcmp(optarg, "odd") == 0)
               parity = MbusSerialMasterProtocol::SER_PARITY_ODD;
            else if (strcmp(optarg, "even") == 0)
               parity = MbusSerialMasterProtocol::SER_PARITY_EVEN;
            else
            {
               port = strtol(optarg, NULL, 0);
               if ((port <= 0) || (port > 0xFFFF))
                  exitBadOption("Invalid parity or port parameter");
            }
         break;
         case 't':
            if (strcmp(optarg, "0") == 0)
               dataType = T0_BIT;
            else if (strcmp(optarg, "1") == 0)
               dataType = T1_BIT;
            else if (strcmp(optarg, "3") == 0)
               dataType = T3_REG16;
            else if (strcmp(optarg, "3:hex") == 0)
               dataType = T3_HEX16;
            else if (strcmp(optarg, "3:int") == 0)
               dataType = T3_INT32;
            else if (strcmp(optarg, "3:mod") == 0)
               dataType = T3_MOD10000;
            else if (strcmp(optarg, "3:float") == 0)
               dataType = T3_FLOAT32;
            else if (strcmp(optarg, "4") == 0)
               dataType = T4_REG16;
            else if (strcmp(optarg, "4:hex") == 0)
               dataType = T4_HEX16;
            else if (strcmp(optarg, "4:int") == 0)
               dataType = T4_INT32;
            else if (strcmp(optarg, "4:mod") == 0)
               dataType = T4_MOD10000;
            else if (strcmp(optarg, "4:float") == 0)
               dataType = T4_FLOAT32;
            else
               exitBadOption("Invalid data type parameter");
         break;
         case 'h':
            printUsage();
         break;
         default:
            exitBadOption("Unrecognized option or missing option parameter");
         break;
      }
   }

   //
   // Read or write count
   //
   writeCnt = argc - optind - 1;
   if (writeCnt < 0)
      exitBadOption("Serialport or host parameter missing");
   if ((writeCnt > 0) && (refCnt != writeCnt) && (refCnt != -1))
      exitBadOption("-c parameter must not be specified for writing");
   if (refCnt == -1) // If not set, set to default of 1
      refCnt = 1;

   if ((dataType == T0_BIT) || (dataType == T1_BIT))
   {
      if ((refCnt <= 0) || (refCnt > MAX_REF_CNT * 16))
         exitBadOption("Invalid count parameter");
   }
   else
   {
      if ((refCnt <= 0) || (refCnt > MAX_REF_CNT))
         exitBadOption("Invalid count parameter");
   }


   portName = argv[optind];

   //
   // No protocol parameter specified, auto-detect
   //
   if (protocol == -1)
   {
#ifdef __WIN32__
      if ((strstr(portName, "com") != NULL) || (strstr(portName, "COM") != NULL)) // Indicates a port name
         protocol = RTU;
      else if (strchr(portName, '\\') != NULL) // Indicates a device
         protocol = RTU;
#else
      if (strchr(portName, '/') != NULL) // Indicates a device
         protocol = RTU;
#endif
      else
         protocol = TCP;
   }

   if ((slaveAddr == 0) && ((protocol == RTU) || (protocol == ASCII)))
   {
       exitBadOption("Invalid slave address parameter");
   }

   //
   // Check for additional parameters which are data to be written
   //
   if (writeCnt > 0)
   {
      switch (dataType)
      {
         case T0_BIT:
            for (i = 0; i < writeCnt; i++)
            {
               char *errPtr;
               long longValue;

               errno = 0;
               longValue = strtol(argv[optind + 1 + i], &errPtr, 10);
               if ((errno != 0) || (*errPtr != '\0') || (longValue < 0) || (longValue > 1))
                  exitBadOption("Illegal write data value");
               intArr[i] = (int) longValue;
            }
         break;
         case T4_REG16:
         case T4_HEX16:
            for (i = 0; i < writeCnt; i++)
            {
               char *errPtr;
               long longValue;

               errno = 0;
               longValue = strtol(argv[optind + 1 + i], &errPtr, 0);
               if ((errno != 0) || (*errPtr != '\0') || (longValue > 0xFFFF) || (longValue < -32768))
                  exitBadOption("Illegal write data value");
               shortArr[i] = (short) longValue;
            }
         break;
         case T4_INT32:
         case T4_MOD10000:
            for (i = 0; i < writeCnt; i++)
            {
               char *errPtr;
               long longValue;

               errno = 0;
               if (argv[optind + 1 + i][0] == '-') // Check for sign
                  longValue = strtol(argv[optind + 1 + i], &errPtr, 0);
               else
                  longValue = (long) strtoul(argv[optind + 1 + i], &errPtr, 0);
               if ((errno != 0) || (*errPtr != '\0'))
                  exitBadOption("Illegal write data value");
               intArr[i] = (int) longValue;
            }
         break;
         case T4_FLOAT32:
            for (i = 0; i < writeCnt; i++)
            {
               char *errPtr;
               double doubleValue;

               errno = 0;
               doubleValue = strtod(argv[optind + 1 + i], &errPtr);
               if ((errno != 0) || (*errPtr != '\0') || (doubleValue > FLT_MAX) || (doubleValue < -FLT_MAX))
                  exitBadOption("Illegal write data value");
               floatArr[i] = (float) doubleValue;
            }
         break;
         break;
         case T1_BIT:
         case T3_REG16:
         case T3_HEX16:
         case T3_FLOAT32:
         case T3_INT32:
            exitBadOption("This data type is read only");
         break;
         default:
            exitBadOption("Write not supported for this data type");
         break;
      }
   }
}


/**
 * Opens protocol and allocates data memory
 */
void openProtocol()
{
   int result = -1;

   switch (protocol)
   {
      case RTU:
         mbusPtr = new MbusRtuMasterProtocol();
         if (swapInts)
            mbusPtr->configureBigEndianInts();
         if (swapFloats)
            mbusPtr->configureSwappedFloats();
         if (enronMode)
            mbusPtr->configureEnron32BitMode();
         if (pduAddressing)
            mbusPtr->configureCountFromZero();
         mbusPtr->setRetryCnt(0);
         mbusPtr->setPollDelay(pollDelay);
         mbusPtr->setTimeout(timeOut);
         if (rs485Mode > 0)
            ((MbusRtuMasterProtocol *) mbusPtr)->enableRs485Mode(rs485Mode);
         result = ((MbusRtuMasterProtocol *) mbusPtr)->openProtocol(
                   portName, baudRate, dataBits, stopBits, parity);
      break;
      case ASCII:
         mbusPtr = new MbusAsciiMasterProtocol();
         if (swapInts)
            mbusPtr->configureBigEndianInts();
         if (swapFloats)
            mbusPtr->configureSwappedFloats();
         if (enronMode)
            mbusPtr->configureEnron32BitMode();
         if (pduAddressing)
            mbusPtr->configureCountFromZero();
         mbusPtr->setRetryCnt(0);
         mbusPtr->setPollDelay(pollDelay);
         mbusPtr->setTimeout(timeOut);
         if (rs485Mode > 0)
            ((MbusAsciiMasterProtocol *) mbusPtr)->enableRs485Mode(rs485Mode);
         result = ((MbusAsciiMasterProtocol *) mbusPtr)->openProtocol(
                   portName, baudRate, dataBits, stopBits, parity);
      break;
      case TCP:
         mbusPtr = new MbusTcpMasterProtocol();
         if (swapInts)
            mbusPtr->configureBigEndianInts();
         if (swapFloats)
            mbusPtr->configureSwappedFloats();
         if (enronMode)
            mbusPtr->configureEnron32BitMode();
         if (pduAddressing)
            mbusPtr->configureCountFromZero();
         mbusPtr->setPollDelay(pollDelay);
         mbusPtr->setTimeout(timeOut);
         ((MbusTcpMasterProtocol *) mbusPtr)->setPort((unsigned short) port);
         result = ((MbusTcpMasterProtocol *) mbusPtr)->openProtocol(portName);
      break;
      case RTUOVERTCP:
         mbusPtr = new MbusRtuOverTcpMasterProtocol();
         if (swapInts)
            mbusPtr->configureBigEndianInts();
         if (swapFloats)
            mbusPtr->configureSwappedFloats();
         if (enronMode)
            mbusPtr->configureEnron32BitMode();
         if (pduAddressing)
            mbusPtr->configureCountFromZero();
         mbusPtr->setPollDelay(pollDelay);
         mbusPtr->setTimeout(timeOut);
         ((MbusRtuOverTcpMasterProtocol *) mbusPtr)->setPort((unsigned short) port);
         result = ((MbusRtuOverTcpMasterProtocol *) mbusPtr)->openProtocol(portName);
      break;
   }
   switch (result)
   {
      case FTALK_SUCCESS:
      break;
      case FTALK_ILLEGAL_ARGUMENT_ERROR:
         fprintf(stderr, "Configuration setting not supported!\n");
         exit(EXIT_FAILURE);
      break;
      case FTALK_TCPIP_CONNECT_ERR:
         fprintf(stderr, "Can't reach server/slave! Check TCP/IP and firewall settings.\n");
         exit(EXIT_FAILURE);
      break;
      default:
         fprintf(stderr, "%s!\n", getBusProtocolErrorText(result));
         exit(EXIT_FAILURE);
      break;
   }
}


/**
 * Closes protocol
 */
void closeProtocol()
{
   delete mbusPtr;
}


/**
 * Poll slave device
 */
int pollSlave()
{
   int i;
   int result = -1;

   while ((pollCnt == -1) || (pollCnt > 0))
   {
      if (pollCnt == -1)
         printf("-- Polling slave... (Ctrl-C to stop)\n");
      else
      {
         printf("-- Polling slave...\n");
         pollCnt--;
      }
      switch (dataType)
      {
         case T0_BIT:
            result = mbusPtr->readCoils(slaveAddr, ref,
                                        intArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: %d\n", ref + i, intArr[i]);
         break;
         case T1_BIT:
            result = mbusPtr->readInputDiscretes(slaveAddr, ref,
                                                 intArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: %d\n", ref + i, intArr[i]);
         break;
         case T4_REG16:
            result = mbusPtr->readMultipleRegisters(slaveAddr, ref,
                                                    shortArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: %hd\n", ref + i, shortArr[i]);
         break;
         case T4_HEX16:
            result = mbusPtr->readMultipleRegisters(slaveAddr, ref,
                                                    shortArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: 0x%04hX\n", ref + i, shortArr[i]);
         break;
         case T4_INT32:
            result = mbusPtr->readMultipleLongInts(slaveAddr, ref,
                                                   intArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: %d\n", ref + i * 2, intArr[i]);
         break;
         case T4_MOD10000:
            result = mbusPtr->readMultipleMod10000(slaveAddr, ref,
                                                   intArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: %d\n", ref + i * 2, intArr[i]);
         break;
         case T4_FLOAT32:
            result = mbusPtr->readMultipleFloats(slaveAddr, ref,
                                                 floatArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: %f\n", ref + i * 2, floatArr[i]);
         break;
         case T3_REG16:
            result = mbusPtr->readInputRegisters(slaveAddr, ref,
                                                 shortArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: %hd\n", ref + i, shortArr[i]);
         break;
         case T3_HEX16:
            result = mbusPtr->readInputRegisters(slaveAddr, ref,
                                                 shortArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: 0x%04hX\n", ref + i, shortArr[i]);
         break;
         case T3_INT32:
            result = mbusPtr->readInputLongInts(slaveAddr, ref,
                                                intArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: %d\n", ref + i * 2, intArr[i]);
         break;
         case T3_MOD10000:
            result = mbusPtr->readInputMod10000(slaveAddr, ref,
                                                intArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: %d\n", ref + i * 2, intArr[i]);
         break;
         case T3_FLOAT32:
            result = mbusPtr->readInputFloats(slaveAddr, ref,
                                              floatArr, refCnt);
            if (result == FTALK_SUCCESS)
               for (i = 0; i < refCnt; i++)
                  printf("[%d]: %f\n", ref + i * 2, floatArr[i]);
         break;
      }
      if (result != FTALK_SUCCESS)
      {
         fprintf(stderr, "%s!\n", getBusProtocolErrorText(result));
         // Stop for fatal errors
         if (!(result & FTALK_BUS_PROTOCOL_ERROR_CLASS))
            return result;
      }
   }

   return result;
}


/**
 * Write to slave device
 */
int writeSlave()
{
   int result = -1;

   switch (dataType)
   {
      case T0_BIT:
         result = mbusPtr->forceMultipleCoils(slaveAddr, ref,
                                              intArr, writeCnt);
      break;
      case T4_REG16:
      case T4_HEX16:
         result = mbusPtr->writeMultipleRegisters(slaveAddr, ref,
                                                  shortArr, writeCnt);
      break;
      case T4_INT32:
         result = mbusPtr->writeMultipleLongInts(slaveAddr, ref,
                                                 intArr, writeCnt);
      break;
      case T4_MOD10000:
         result = mbusPtr->writeMultipleMod10000(slaveAddr, ref,
                                                 intArr, writeCnt);
      break;
      case T4_FLOAT32:
         result = mbusPtr->writeMultipleFloats(slaveAddr, ref,
                                               floatArr, writeCnt);
      break;
   }

   if (result != FTALK_SUCCESS)
      fprintf(stderr, "%s!\n", getBusProtocolErrorText(result));
   else
      printf("Written %d reference%s.\n", writeCnt, writeCnt == 1 ? "" : "s");

   return result;
}


/**
 * Main function
 *
 * @param argc Command line argument count
 * @param argv Command line argument value string array
 * @return Error code: 0 = OK, else error
 */
int main (int argc, char **argv)
{
   int result;

   scanOptions(argc, argv);
   printConfig();
   atexit(closeProtocol);
   openProtocol();

   // Additional parameters given for writing ?
   if (writeCnt > 0)
      result = writeSlave();
   else
      result = pollSlave();

   if (result == FTALK_SUCCESS)
      return (EXIT_SUCCESS);
   else
      return (EXIT_FAILURE);
}
