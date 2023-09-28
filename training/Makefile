prog: training.cpp
	g++ -DUSE_AVX_INSTRUCTIONS=ON -O3 -o prog training.cpp `pkg-config opencv4 --cflags --libs` -pthread
