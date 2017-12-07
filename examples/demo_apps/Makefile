all: sender receiver

sender: 
	g++ mshcs_sender.cpp -o sender.out -l boost_system -l boost_chrono -l boost_thread
receiver: 
	g++ mshcs_receiver.cpp -o receiver.out -l boost_system -l boost_chrono -l boost_thread
	
clean:
	rm -f  *.out
