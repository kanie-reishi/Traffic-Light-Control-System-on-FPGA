#!/bin/bash
# ============================================================================
# run.sh - Build and run the UART Density Sender
# Usage:   sh run.sh COM3
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC="$SCRIPT_DIR/uart_density_sender.cpp"
EXE="$SCRIPT_DIR/uart_density_sender.exe"

# --- Build ---
echo ""
echo "  [BUILD] Compiling uart_density_sender.cpp ..."
g++ -o "$EXE" "$SRC"

if [ $? -ne 0 ]; then
    echo "  [ERROR] Compilation failed!"
    exit 1
fi
echo "  [BUILD] OK -> uart_density_sender.exe"

# --- Run ---
if [ -z "$1" ]; then
    echo ""
    echo "  Usage: sh run.sh <COM_PORT>"
    echo "  Example: sh run.sh COM3"
    echo ""
    exit 1
fi

echo ""
exec "$EXE" "$1"
