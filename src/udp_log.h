#ifndef UDP_LOG_H
#define UDP_LOG_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdarg.h>

#define UDP_LOG_PORT 14514

static HardwareSerial& SerialNative = Serial;
static WiFiUDP udpLog;

class UdpMirrorSerialClass : public Stream {
public:
    void begin(unsigned long baud) {
        SerialNative.begin(baud);
    }

    void begin(unsigned long baud, uint32_t config) {
        SerialNative.begin(baud, config);
    }

    operator bool() {
        return (bool)SerialNative;
    }

    void flush() {
        SerialNative.flush();
    }

    int available() {
        return SerialNative.available();
    }

    int read() {
        return SerialNative.read();
    }

    int peek() {
        return SerialNative.peek();
    }

    size_t write(uint8_t value) override {
        size_t written = SerialNative.write(value);
        mirrorBytes(&value, 1);
        return written;
    }

    size_t write(const uint8_t* buffer, size_t size) override {
        size_t written = SerialNative.write(buffer, size);
        mirrorBytes(buffer, size);
        return written;
    }

    using Print::write;

    int printf(const char* format, ...) {
        char buffer[512];
        va_list args;
        va_start(args, format);
        int len = vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        if (len > 0) {
            write(reinterpret_cast<const uint8_t*>(buffer), strnlen(buffer, sizeof(buffer)));
        }
        return len;
    }

private:
    void mirrorBytes(const uint8_t* buffer, size_t size) {
        if (size == 0 || !WiFi.isConnected()) {
            return;
        }

        IPAddress local_ip = WiFi.localIP();
        IPAddress subnet = WiFi.subnetMask();
        if (local_ip == INADDR_NONE || subnet == INADDR_NONE) {
            return;
        }

        IPAddress broadcast;
        for (int i = 0; i < 4; ++i) {
            broadcast[i] = local_ip[i] | (~subnet[i]);
        }

        if (udpLog.beginPacket(broadcast, UDP_LOG_PORT)) {
            udpLog.write(buffer, size);
            udpLog.endPacket();
        }
    }
};

static UdpMirrorSerialClass UdpMirrorSerial;

inline void initUdpLog() {
    SerialNative.printf("[UDP] Broadcast log stream ready on port %d\n", UDP_LOG_PORT);
}

#define Serial UdpMirrorSerial

#endif // UDP_LOG_H
