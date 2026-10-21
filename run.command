#!/bin/bash
cd "$(dirname "$0")"
echo "Compiling Smart Route Optimizer 2.0..."
c++ -std=c++17 -O2 optimizer.cpp -o optimizer || exit 1
(sleep 1; open http://localhost:8080) &
python3 server.py
