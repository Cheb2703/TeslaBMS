#include <Arduino.h>
#include "Logger.h"

class BMSUtil {    
public:
    
    static uint8_t genCRC(uint8_t *input, int lenInput)
    {
        uint8_t generator = 0x07;
        uint8_t crc = 0;

        for (int x = 0; x < lenInput; x++)
        {
            crc ^= input[x]; /* XOR-in the next input byte */

            for (int i = 0; i < 8; i++)
            {
                if ((crc & 0x80) != 0)
                {
                    crc = (uint8_t)((crc << 1) ^ generator);
                }
                else
                {
                    crc <<= 1;
                }
            }
        }

        return crc;
    }

    static void sendData(uint8_t *data, uint8_t dataLen, bool isWrite)
    {
        uint8_t orig = data[0];
        uint8_t addrByte = data[0];
        if (isWrite) addrByte |= 1;
        SERIAL.write(addrByte);
        SERIAL.write(&data[1], dataLen - 1);  //assumes that there are at least 2 bytes sent every time. There should be, addr and cmd at the least.
        data[0] = addrByte;
        if (isWrite) SERIAL.write(genCRC(data, dataLen));        

        if (Logger::isDebug())
        {
            SERIALCONSOLE.print("Sending: ");
            SERIALCONSOLE.print(addrByte, HEX);
            SERIALCONSOLE.print(" ");
            for (int x = 1; x < dataLen; x++) {
                SERIALCONSOLE.print(data[x], HEX);
                SERIALCONSOLE.print(" ");
            }
            if (isWrite) SERIALCONSOLE.print(genCRC(data, dataLen), HEX);
            SERIALCONSOLE.println();
        }
        
        data[0] = orig;
    }

    static int getReply(uint8_t *data, int maxLen)
    { 
        int numBytes = 0; 
        if (Logger::isDebug()) SERIALCONSOLE.print("Reply: ");

        // Wait for bytes to actually arrive instead of taking a single
        // snapshot of SERIAL.available(). On a busy loop (display refresh,
        // WiFi/OTA, etc.) the scheduler can delay us by a few ms, and a
        // one-shot check can catch the UART mid-transmission and return a
        // short/garbled read. This polls for up to ~15ms total, which is
        // generous relative to the ~0.35ms it takes to transmit a 22-byte
        // reply at BMS_BAUD, but still short enough not to stall the loop.
        uint32_t deadline = millis() + 15;
        while (numBytes < maxLen && (int32_t)(millis() - deadline) < 0)
        {
            if (SERIAL.available())
            {
                data[numBytes] = SERIAL.read();
                if (Logger::isDebug()) {
                    SERIALCONSOLE.print(data[numBytes], HEX);
                    SERIALCONSOLE.print(" ");
                }
                numBytes++;
            }
        }
        if (maxLen == numBytes)
        {
            while (SERIAL.available()) SERIAL.read();
        }
        if (Logger::isDebug()) SERIALCONSOLE.println();
        return numBytes;
    }

    //Uses above functions to send data then get the response. Will auto retry if response not
    //the expected return length. This helps to alleviate any comm issues. The Due cannot exactly
    //match the correct comm speed so sometimes there are data glitches.
    static int sendDataWithReply(uint8_t *data, uint8_t dataLen, bool isWrite, uint8_t *retData, int retLen)
    {
        int attempts = 1;
        int returnedLength;
        while (attempts < 4)
        {
            sendData(data, dataLen, isWrite);
            // Small settle delay before polling for the reply. getReply()
            // itself now waits (bounded) for bytes to arrive, so this no
            // longer needs to be sized to cover the whole transfer -- it's
            // just giving the BMB a moment to start responding.
            delay(3);
            returnedLength = getReply(retData, retLen);
            if (returnedLength == retLen) return returnedLength;
            attempts++;
        }
        return returnedLength; //failed to get a proper response.
    }
};