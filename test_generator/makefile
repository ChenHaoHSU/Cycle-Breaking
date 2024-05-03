BINARY = test_generator
CFLAGS = -std=c++11 -O3 -static -Wall
#CFLAGS = -std=c++11 -O3 -static -Wall -DDEBUG

$(BINARY): test_generator.cpp
	g++ $< -o $@ $(CFLAGS)

clean:
	rm -rf $(BINARY)
