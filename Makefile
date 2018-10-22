all:
	gcc -o TrafficSim TrafficSimulationCPU.c -lm

clean:
	rm -rf  ./log/logger.o  ./TrafficSimulationCPU.o  ./TrafficSimulationCPU.d  ./log/logger.d  TrafficSim

