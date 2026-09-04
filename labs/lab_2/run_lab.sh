#!/usr/bin/env bash
set -e

echo "=== Building Lab 2 C++ Executable ==="
mkdir -p build
cd build
cmake ..
cmake --build .

echo ""
echo "=== Running Lab 2 C++ Discrete Simulation ==="
./lab2_sim > cpp_output.csv

echo ""
echo "=== Output Sample (First 6 Rows) ==="
head -n 6 cpp_output.csv

echo ""
echo "Lab 2 Execution Complete. Executable at: labs/lab_2/build/lab2_sim"
