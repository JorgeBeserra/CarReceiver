#!/usr/bin/env bash
set -euo pipefail
REPO_URL=${REPO_URL:-https://github.com/jorgebeserra/carreceiver.git}
BUILD_CMD=${BUILD_CMD:-pio run}
DEVICE=${DEVICE:-esp32}

# Simple backup
BACKUP_DIR="/tmp/carreceiver-backup-$(date +%Y%m%d%H%M%S)"
mkdir -p "$BACKUP_DIR"
rsync -av --delete /home/jorge/.openclaw/workspace/CarReceiver "$BACKUP_DIR/" >/dev/null 2>&1 || true

# Fetch latest
cd /home/jorge/.openclaw/workspace/CarReceiver
git fetch origin main
git reset --hard origin/main

# Build firmware
$BUILD_CMD

# Optional OTA logic would go here

echo "Update complete. Backup stored at $BACKUP_DIR"
