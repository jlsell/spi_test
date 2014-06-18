CXX=g++
CPPFLAGS=-std=c++0x 
LIBS=-lbcm2835

all: imutest

imutest: NEWimutest1burst.o
	$(CXX) $(CPPFLAGS) NEWimutest1burst.o -oimutest $(LIBS)

NEWimutest1burst.o: NEWimutest1burst.cpp
	$(CXX) $(CPPFLAGS) -c NEWimutest1burst.cpp

install: imutest
	update-rc.d -f imu.sh remove
	cp -f imutest /usr/bin/
	cp -f imu.sh /etc/init.d/
	chmod 755 /etc/init.d/imu.sh
	update-rc.d imu.sh defaults

clean:
	rm -rf NEWimutest1burst.o imutest
