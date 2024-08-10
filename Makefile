all: compile link

compile:
	g++ -c main.cpp -IC:\Users\Gia-Minh\projects\libraries\SFML-2.6.1\include

link:
	g++ main.o -o main -LC:\Users\Gia-Minh\projects\libraries\SFML-2.6.1\lib -lmingw32 -lsfml-graphics -lsfml-window -lsfml-system -lsfml-main 
#-mwindows

run: all
	./main.exe

clean:
	rm -f main *.o