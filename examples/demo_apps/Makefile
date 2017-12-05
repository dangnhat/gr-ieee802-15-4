all: sender receiver

sender: 
	g++ mshcs_sender.cpp -o sender.out -l boost_system -l boost_chrono
receiver: 
	g++ mshcs_receiver.cpp -o receiver.out -l boost_system -l boost_chrono
	
clean:
	rm -f  *.out
