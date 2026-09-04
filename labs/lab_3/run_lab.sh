#!/usr/bin/env bash
set -e

echo "=== Building Lab 3 C++ Executable ==="
mkdir -p build
cd build
cmake ..
cmake --build .

echo ""
echo "=== Running Lab 3 C++ PID Control Simulation ==="
./lab3_sim > cpp_output.csv

echo ""
echo "=== Output Sample (First 6 Rows) ==="
head -n 6 cpp_output.csv

echo ""
echo "Lab 3 Execution Complete. Executable at: labs/lab_3/build/lab3_sim"
