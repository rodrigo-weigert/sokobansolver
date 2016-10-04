all: sokoban

sokoban:
	g++ sokoban.cpp -Wall -O2 -o sokoban

.PHONY: clear

clear:
	rm -f sokoban
