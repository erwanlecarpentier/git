CCC=g++
CCFLAGS=-std=c++11 -Wall -Wextra -I. -O2 -s -g #-ansi -pedantic
LDFLAGS=-lm -lconfig++
EXEC=main
EXEC2=FZmain

.PHONY : all compile run clean

all : clean compile run

run : clean_data
	./$(EXEC)

compile : demo/main.cpp L2Fsim/flight_zone/flat_thermal_soaring_zone.cpp L2Fsim/flight_zone/thermal/std_thermal.cpp
	$(CCC) $(CCFLAGS) demo/main.cpp L2Fsim/flight_zone/flat_thermal_soaring_zone.cpp L2Fsim/flight_zone/thermal/std_thermal.cpp -o $(EXEC) $(LDFLAGS)

therm :
	./$(EXEC2)

compile2 : demo/FZmain.cpp L2Fsim/flight_zone/flat_thermal_soaring_zone.cpp L2Fsim/flight_zone/thermal/std_thermal.cpp
	$(CCC) $(CCFLAGS) demo/FZmain.cpp L2Fsim/flight_zone/flat_thermal_soaring_zone.cpp L2Fsim/flight_zone/thermal/std_thermal.cpp -o $(EXEC2) $(LDFLAGS)

thermvis :
	python3 plot/therm_zslice.py

thermscat :
	python3 plot/thermalscatter.py

2d_trajectory :
	python3 plot/2d_trajectory.py

3d_trajectory :
	python3 plot/3d_trajectory.py

variables :
	python3 plot/variables.py

plot_all :
	python3 plot/2d_trajectory.py & python3 plot/3d_trajectory.py & python3 plot/variables.py

clean :
	rm -f data/state.dat
	rm -f data/wind.dat
	rm -f data/config.txt
	rm -f $(EXEC)
	rm -f $(EXEC2)

clean_data :
	rm -f data/state.dat
	rm -f data/wind.dat
	rm -f data/config.txt

help :
	@echo “compile : compile “demo/main.cpp”, executable is $(EXEC)”
	@echo “run : clean and run $(EXEC)”
	@echo “all : clean, compile and run $(EXEC)”
	@echo “therm : compile and run $(EXEC2)”
	@echo “thermvis : run therm_zslice.py”
	@echo “2d_trajectory : run 2d_trajectory.py”
	@echo “3d_trajectory : run 3d_trajectory.py”
	@echo “clean : remove executable and data files”
	@echo “clean_data : remove data files”

