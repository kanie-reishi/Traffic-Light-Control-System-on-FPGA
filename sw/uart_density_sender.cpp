// ============================================================================
// uart_density_sender.cpp
// PC-side UART driver for Traffic Light Control System on FPGA
//
// Sends density packets to the FPGA's uart_camera_rx module over COM port.
// Packet format: 0xAA | [NS_density(7:4) | EW_density(3:0)] | 0x55
// UART config:   115200 baud, 8N1
//
// Build (MSVC):   cl /EHsc uart_density_sender.cpp
// Build (MinGW):  g++ -o uart_density_sender.exe uart_density_sender.cpp
// Usage:          uart_density_sender.exe COM3
// ============================================================================

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <conio.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

// ============================================================================
// Serial port helpers
// ============================================================================
HANDLE openSerialPort(const char* portName) {
    // Prepend \\.\  for COM port numbers >= 10 compatibility
    std::string fullName = std::string("\\\\.\\") + portName;

    HANDLE hSerial = CreateFileA(
        fullName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    if (hSerial == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "[ERROR] Cannot open %s (error %lu)\n", portName, GetLastError());
        return INVALID_HANDLE_VALUE;
    }

    // Configure: 115200 baud, 8 data bits, no parity, 1 stop bit
    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(hSerial, &dcb)) {
        fprintf(stderr, "[ERROR] GetCommState failed\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(hSerial, &dcb)) {
        fprintf(stderr, "[ERROR] SetCommState failed\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    // Set timeouts (non-blocking reads, blocking writes)
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial, &timeouts);

    return hSerial;
}

bool sendPacket(HANDLE hSerial, int ns_density, int ew_density) {
    unsigned char packet[3];
    packet[0] = 0xAA;                                        // start marker
    packet[1] = ((ns_density & 0x0F) << 4) | (ew_density & 0x0F); // density byte
    packet[2] = 0x55;                                        // end marker

    DWORD bytesWritten = 0;
    if (!WriteFile(hSerial, packet, 3, &bytesWritten, NULL) || bytesWritten != 3) {
        fprintf(stderr, "[ERROR] WriteFile failed (error %lu)\n", GetLastError());
        return false;
    }
    return true;
}

// ============================================================================
// Display helpers
// ============================================================================
void printBanner() {
    printf("\n");
    printf("  ====================================================\n");
    printf("  Traffic Light UART Density Sender\n");
    printf("  Protocol: 0xAA | [NS<<4 | EW] | 0x55  @ 115200 8N1\n");
    printf("  ====================================================\n\n");
}

void printHelp() {
    printf("  Commands:\n");
    printf("    <NS> <EW>        Send density packet (values 0-15)\n");
    printf("                     Example: 12 3\n");
    printf("    auto <NS> <EW> <ms>  Continuous mode: send every <ms> milliseconds\n");
    printf("                     Example: auto 10 5 200\n");
    printf("    stop             Stop continuous mode\n");
    printf("    help             Show this message\n");
    printf("    quit / exit      Exit program\n\n");
}

void printDensityBar(const char* label, int density) {
    printf("    %s [", label);
    for (int i = 0; i < 15; i++)
        printf("%c", i < density ? '#' : '.');
    printf("] %2d/15\n", density);
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char* argv[]) {
    printBanner();

    if (argc < 2) {
        printf("  Usage: %s <COM_PORT>\n", argv[0]);
        printf("  Example: %s COM3\n\n", argv[0]);
        return 1;
    }

    const char* portName = argv[1];
    printf("  Opening %s ...\n", portName);

    HANDLE hSerial = openSerialPort(portName);
    if (hSerial == INVALID_HANDLE_VALUE)
        return 1;

    printf("  [OK] %s opened at 115200 baud, 8N1\n\n", portName);
    printHelp();

    // Auto-send state
    bool autoMode = false;
    int autoNS = 0, autoEW = 0, autoIntervalMs = 200;
    DWORD lastAutoSend = 0;

    char inputBuf[256];

    while (true) {
        // In auto mode, send packets at the specified interval
        if (autoMode) {
            DWORD now = GetTickCount();
            if (now - lastAutoSend >= (DWORD)autoIntervalMs) {
                if (sendPacket(hSerial, autoNS, autoEW)) {
                    printf("\r  [AUTO] NS=%2d EW=%2d (every %dms)  -- type 'stop' to end",
                           autoNS, autoEW, autoIntervalMs);
                    fflush(stdout);
                }
                lastAutoSend = now;
            }

            // Check for keyboard input (non-blocking)
            // Use _kbhit on Windows to avoid blocking in auto mode
            if (_kbhit()) {
                printf("\n");
                if (fgets(inputBuf, sizeof(inputBuf), stdin)) {
                    if (strncmp(inputBuf, "stop", 4) == 0) {
                        autoMode = false;
                        printf("  [STOPPED] Auto mode disabled\n\n");
                        printf("> ");
                    } else if (strncmp(inputBuf, "quit", 4) == 0 ||
                               strncmp(inputBuf, "exit", 4) == 0) {
                        break;
                    }
                }
            } else {
                Sleep(10); // avoid busy-wait
            }
            continue;
        }

        // Normal interactive mode
        printf("> ");
        fflush(stdout);

        if (!fgets(inputBuf, sizeof(inputBuf), stdin))
            break;

        // Trim newline
        inputBuf[strcspn(inputBuf, "\r\n")] = '\0';

        // Parse commands
        if (strlen(inputBuf) == 0)
            continue;

        if (strcmp(inputBuf, "quit") == 0 || strcmp(inputBuf, "exit") == 0)
            break;

        if (strcmp(inputBuf, "help") == 0) {
            printHelp();
            continue;
        }

        // Auto mode: "auto <NS> <EW> <ms>"
        if (strncmp(inputBuf, "auto", 4) == 0) {
            int ns, ew, ms;
            if (sscanf(inputBuf + 4, "%d %d %d", &ns, &ew, &ms) == 3) {
                if (ns < 0 || ns > 15 || ew < 0 || ew > 15) {
                    printf("  [ERROR] Density values must be 0-15\n");
                    continue;
                }
                if (ms < 50) {
                    printf("  [ERROR] Interval must be >= 50ms\n");
                    continue;
                }
                autoMode = true;
                autoNS = ns;
                autoEW = ew;
                autoIntervalMs = ms;
                lastAutoSend = 0;
                printf("  [AUTO] Starting: NS=%d, EW=%d, interval=%dms\n", ns, ew, ms);
                printf("  Type 'stop' + Enter to return to manual mode\n");
            } else {
                printf("  Usage: auto <NS> <EW> <interval_ms>\n");
            }
            continue;
        }

        // Manual send: "<NS> <EW>"
        int ns, ew;
        if (sscanf(inputBuf, "%d %d", &ns, &ew) == 2) {
            if (ns < 0 || ns > 15 || ew < 0 || ew > 15) {
                printf("  [ERROR] Density values must be 0-15\n");
                continue;
            }

            if (sendPacket(hSerial, ns, ew)) {
                printf("  [SENT] Packet: 0xAA 0x%02X 0x55\n",
                       ((ns & 0x0F) << 4) | (ew & 0x0F));
                printDensityBar("NS", ns);
                printDensityBar("EW", ew);
                printf("\n");
            }
        } else {
            printf("  Unknown command. Type 'help' for usage.\n");
        }
    }

    printf("\n  Closing %s. Goodbye!\n", portName);
    CloseHandle(hSerial);
    return 0;
}
