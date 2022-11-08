<<<<<<< HEAD
g++ -c -O3 ParticleFilters.c prob.c
g++ -no-pie *.o -O3 -g -lGL -lGLU -lglut -o ParticleFilters
=======
g++ -c -O3 ParticleFilters.c
g++  *.o -O3 -g -lGL -lGLU -no-pie -lglut -o ParticleFilters
>>>>>>> 7dc5c40137cf5148cb17b6ba24c97bec199f9084

