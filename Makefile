all:consumer producer

CPPFLAGS+=-std=c++11 -Wall -pedantic
CPPFLAGS+=-g -O0
LDFLAGS+=-lboost_system -lrt -lpthread

%:%.cpp
	$(CXX) $(CPPFLAGS) $^ -o $@ $(LDFLAGS)

.PHONY: clean
clean:
	rm consumer
	rm producer
