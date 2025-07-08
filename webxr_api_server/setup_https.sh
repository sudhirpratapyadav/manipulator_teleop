#!/bin/bash

# ANSI color codes
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== WebXR HTTPS Server Setup ===${NC}"

# Check if OpenSSL is installed
if ! command -v openssl &> /dev/null; then
    echo -e "${RED}OpenSSL is not installed. Please install it first.${NC}"
    exit 1
fi

# Create a certificates directory if it doesn't exist
mkdir -p certs
cd certs

echo -e "${BLUE}Generating self-signed certificate for HTTPS...${NC}"

# Generate a private key
openssl genrsa -out key.pem 2048

# Generate a self-signed certificate
openssl req -new -x509 -key key.pem -out cert.pem -days 365 -subj "/CN=localhost" -addext "subjectAltName = IP:127.0.0.1"

cd ..

echo -e "${GREEN}Certificate generated successfully!${NC}"

