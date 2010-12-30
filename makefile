all: main

main:
	g++ -o boids boids.cpp -lglut -lGL -lGLU -L/usr/X11R6/lib -lX11 -lm -pthread -ggdb -Wall
	rm *.o -f
	rm *~ -f

boids.o : boids.cpp
	 g++ -c boids.cpp

clean:
	rm *.o -f
	rm *~ -f
	rm boids -f
