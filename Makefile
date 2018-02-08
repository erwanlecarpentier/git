CCC=g++
INCLUDE = -I./src -I./src/aircraft -I./src/flight_zone -I./src/pilot -I./src/stepper -I./src/utils
CCFLAGS=-std=c++11 -Wall -Wextra -I. ${INCLUDE} -O2 -g
LDFLAGS=-lm -lconfig++ #-s
EXEC=main
MAIN_CPP=demo/main.cpp

.PHONY : all compile run clean

all : clean compile run

run :
	./${EXEC}

compile : ${MAIN_CPP}
	${CCC} ${CCFLAGS} ${MAIN_CPP} -o ${EXEC} ${LDFLAGS}

thermal_magnitude :
	python3 plot/thermal_magnitude.py

2d_trajectory :
	python3 plot/2d_trajectory.py

3d_trajectory :
	python3 plot/3d_trajectory.py

variables :
	python3 plot/variables.py

plot_all :
	python3 plot/2d_trajectory.py & python3 plot/3d_trajectory.py & python3 plot/variables.py

clean : clean_exe clean_dat

clean_exe :
	rm -f ${EXEC}

clean_dat :
	rm -f data/state.dat
	rm -f data/wind.dat
	rm -f data/updraft_field.dat

help :
	@echo ----------------------------------------------------------------------
	@echo Learning to fly project
	@echo Help section
	@echo ----------------------------------------------------------------------
	@echo
	@echo Makefile commands:
	@echo
	@echo - General:
	@echo compile : compile ”${MAIN_CPP}”, executable is ”${EXEC}”
	@echo run     : execute ”${EXEC}”
	@echo all     : clean, compile and execute ”${EXEC}”
	@echo
	@echo - Plot:
	@echo plot              : plot 2D, 3D trajectories and variables
	@echo thermal_magnitude : plot thermals
	@echo 2d_trajectory     : plot 2D trajectory (seen from above)
	@echo 3d_trajectory     : plot 3D trajectory
	@echo variables         : plot flight variables
	@echo
	@echo - Clean:
	@echo clean : remove executables and data files
	@echo clean_exe : remove executables
	@echo clean_dat : remove data files
	@echo
	@echo ----------------------------------------------------------------------

