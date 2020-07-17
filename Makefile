all:ShmClient ShmServer

CPPFLAGS+=-std=c++1 -Wall -pedantic
CPPFLAGS+=-g -O0
LDFLAGS+=-lboost_system -lrt -lpthread

%:%.cpp
    $(CXX) $(CPPFLAGS) $^ -o $@ $(LDFLAGS)

.PHONY: clean
clean:
    rm ShmClient
    rm ShmServer
