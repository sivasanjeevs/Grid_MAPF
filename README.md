## Build Instructions
./compile.sh

or

cd build
cmake ..
make -j4


## Running the Program (10x10 grid)
./build/lifelong --inputFile ./example_problems/random.domain/random_32_32_20_100.json -o test.json


## Visualization 🗺️
cd PlanViz-main

python3 script/run.py --map ../example_problems/random.domain/maps/random-32-32-20.map --plan ../test.json --grid --aid --tid

