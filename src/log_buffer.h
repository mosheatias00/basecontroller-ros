#ifndef LOG_BUFFER_H
#define LOG_BUFFER_H

#include <Arduino.h>

// Simple circular buffer for storing recent log messages
#define LOG_BUFFER_SIZE 2048  // Total buffer size in bytes
#define LOG_MAX_LINES 100     // Maximum number of lines to keep

class LogBuffer {
private:
    char buffer[LOG_BUFFER_SIZE];
    uint16_t write_pos = 0;
    uint16_t line_count = 0;
    
public:
    // Add a log message line to the buffer
    void addLog(const char* message) {
        if (!message || message[0] == '\0') return;
        
        uint16_t msg_len = strlen(message);
        
        // If message is too long for remaining space, wrap around
        if (write_pos + msg_len + 1 >= LOG_BUFFER_SIZE) {
            write_pos = 0;
            line_count = 0;
        }
        
        // Add message
        strcpy(&buffer[write_pos], message);
        write_pos += msg_len;
        
        // Add newline if not present
        if (buffer[write_pos - 1] != '\n') {
            buffer[write_pos] = '\n';
            write_pos++;
        }
        
        buffer[write_pos] = '\0';
        line_count++;
        
        // Prevent buffer from getting too fragmented
        if (line_count > LOG_MAX_LINES) {
            compact();
        }
    }
    
    // Get all current logs as a string
    const char* getLogs() {
        return buffer;
    }
    
    // Clear all logs
    void clear() {
        buffer[0] = '\0';
        write_pos = 0;
        line_count = 0;
    }
    
private:
    // Remove oldest lines when buffer gets full
    void compact() {
        // Find first newline and remove everything before it
        char* first_newline = strchr(buffer, '\n');
        if (first_newline && first_newline != buffer) {
            uint16_t offset = first_newline - buffer + 1;
            memmove(buffer, first_newline + 1, write_pos - offset);
            write_pos -= offset;
            buffer[write_pos] = '\0';
            line_count--;
        }
    }
};

// Global log buffer instance
LogBuffer logBuffer;

#endif // LOG_BUFFER_H
