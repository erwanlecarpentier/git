CCC=g++
CCFLAGS=-std=c++11 -Wall -Wextra -I. -O2 -s -g
LDFLAGS=-lm -lconfig++
EXEC=main
MAIN_CPP=demo/main.cpp

.PHONY : all compile run clean

all : clean_exe compile run

run : clean_dat
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

clean_all :
	rm -f data/state.dat
	rm -f data/wind.dat
	rm -f data/updraft_field.dat
	rm -f ${EXEC}

clean_exe :
	rm -f ${EXEC}

clean_dat :
	rm -f data/state.dat
	rm -f data/wind.dat
	rm -f data/updraft_field.dat

help :
	@echo Learning to fly project - Help section:
	@echo
	@echo General:
	@echo compile : compile ”${MAIN_CPP}”, executable is ”${EXEC}”
	@echo run : clean_dat and execute ”${EXEC}”
	@echo all : clean_exe, compile, clean_dat and execute ”${EXEC}”
	@echo
	@echo - Plot:
	@echo thermal_magnitude : run ”/plot/thermal_magnitude.py” with python3
	@echo 2d_trajectory : run ”/plot/2d_trajectory.py” with python3
	@echo 3d_trajectory : run ”/plot/3d_trajectory.py” with python3
	@echo variables : run ”/plot/variables.py” with python3
	@echo
	@echo - Clean:
	@echo clean_all : remove executables and data files
	@echo clean_exe : remove executables
	@echo clean_dat : remove data files
	@echo

