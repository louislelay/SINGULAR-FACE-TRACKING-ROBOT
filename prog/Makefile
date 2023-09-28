prog: prog.cpp
	g++ -DUSE_AVX_INSTRUCTIONS=ON -O3 -o prog prog.cpp `pkg-config opencv4 --cflags --libs` -pthread
