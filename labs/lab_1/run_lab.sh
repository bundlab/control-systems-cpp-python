#!/usr/bin/env bash
set -e

echo "=== Building Chapter 1 C++ Executable ==="
mkdir -p build
cd build
cmake ..
cmake --build .

echo ""
echo "=== Running C++ Simulation ==="
./chapter1_sim > cpp_output.csv

echo ""
echo "=== Running Python Simulation Prototype ==="
cd ..
python3 simulate.py > build/py_output.csv

echo ""
echo "=== Comparing Results (First 5 Rows) ==="
echo "--- C++ Output ---"
head -n 6 build/cpp_output.csv
echo "--- Python Output ---"
head -n 6 build/py_output.csv

echo ""
echo "Lab 1 Execution Complete. Executable available at: build/chapter1_sim"