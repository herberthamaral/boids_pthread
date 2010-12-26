all: main

main: boids.o

	gcc boids.o -o boids  -lglut -lGL -lGLU -L/usr/X11R6/lib -lX11 -lm
	rm *.o -f
	rm *~ -f

boids.o : boids.cpp
	 gcc -c boids.cpp

clean:
	rm *.o -f
	rm *~ -f
	rm boids -f
