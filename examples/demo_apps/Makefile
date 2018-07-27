all: sender receiver suc_app

sender: 
	g++ mshcs_sender.cpp -o sender.out -l boost_system -l boost_chrono -l boost_thread
receiver: 
	g++ mshcs_receiver.cpp -o receiver.out -l boost_system -l boost_chrono -l boost_thread
suc_app:
	g++ ngrc_suc_app.cpp -o suc_app.out 
	
clean:
	rm -f  *.out
