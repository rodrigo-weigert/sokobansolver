all: sokoban

sokoban: sokoban.cpp
	g++ sokoban.cpp -Wall -O3 -o sokoban

.PHONY: clear

clear:
	rm -f sokoban
