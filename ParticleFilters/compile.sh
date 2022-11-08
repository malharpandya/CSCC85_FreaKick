g++ -c -O3 ParticleFilters.c prob.c
g++ -no-pie *.o -O3 -g -lGL -lGLU -lglut -o ParticleFilters

