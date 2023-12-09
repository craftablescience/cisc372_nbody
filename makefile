FLAGS= -DDEBUG
LIBS= -lm
ALWAYS_REBUILD=makefile

.PHONY: all
all: nbody nbody_old

nbody: nbody.o compute.o
	nvcc $(FLAGS) $^ -o $@ $(LIBS)
nbody.o: nbody.cu planets.h config.h vector.h $(ALWAYS_REBUILD)
	nvcc $(FLAGS) -c $<
compute.o: compute.cu config.h vector.h $(ALWAYS_REBUILD)
	nvcc $(FLAGS) -c $<

nbody_old: nbody_old.o compute_old.o
	gcc $(FLAGS) $^ -o $@ $(LIBS)
nbody_old.o: nbody_old.c planets.h config.h vector.h $(ALWAYS_REBUILD)
	gcc $(FLAGS) -c $<
compute_old.o: compute_old.c config.h vector.h $(ALWAYS_REBUILD)
	gcc $(FLAGS) -c $<

clean:
	rm -f *.o nbody nbody_old
