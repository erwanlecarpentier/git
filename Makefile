CCC=g++
CCFLAGS=-std=c++11 -Wall -Wextra -I. -O2 -s -g #-ansi -pedantic
LDFLAGS=-lm -lconfig++
EXEC=main
EXEC2=FZmain

.PHONY : all compile run clean

all : clean_exe compile run

all2 : clean_exe compile2 run2

run : clean_dat
	./$(EXEC)

compile : demo/main.cpp
	$(CCC) $(CCFLAGS) demo/main.cpp -o $(EXEC) $(LDFLAGS)

run2 :
	./$(EXEC2)

compile2 : demo/FZmain.cpp
	$(CCC) $(CCFLAGS) demo/FZmain.cpp -o $(EXEC2) $(LDFLAGS)

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
	rm -f $(EXEC)
	rm -f $(EXEC2)

clean_exe :
	rm -f $(EXEC)
	rm -f $(EXEC2)

clean_dat :
	rm -f data/state.dat
	rm -f data/wind.dat
	rm -f data/updraft_field.dat

help :
	@echo “compile : compile “demo/main.cpp”, executable is $(EXEC)”
	@echo “run : clean and run $(EXEC)”
	@echo “all : clean, compile and run $(EXEC)”
	@echo “run2 : compile and run $(EXEC2)”
	@echo “thermal_magnitude : run thermal_magnitude.py”
	@echo “2d_trajectory : run 2d_trajectory.py”
	@echo “3d_trajectory : run 3d_trajectory.py”
	@echo “clean_all : remove executables and data files”
	@echo “clean_exe : remove executables”
	@echo “clean_dat : remove data files”

