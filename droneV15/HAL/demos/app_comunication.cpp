#include <unistd.h>
#include <iostream>

#include "../cpp/driver/everloop_image.h"
#include "../cpp/driver/everloop.h"
#include "../cpp/driver/wishbone_bus.h"

#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>



namespace hal = matrix_hal;

int main() {
  hal::WishboneBus bus;

  bus.SpiInit();

  hal::Everloop everloop;
  hal::EverloopImage image1d;

  everloop.Setup(&bus);

  char led=0;
  ////////////SET UP COMUNICATION////////////////
    int client, server;
    int portNum = 1500;
    bool isExit = false;
    int bufsize = 1024;
    char buffer[bufsize];

    struct sockaddr_in server_addr;
    socklen_t size;

    client = socket(AF_INET, SOCK_STREAM, 0);

    if (client < 0) 
    {
        cout << "\nError establishing socket..." << endl;
        exit(1);
    }

    cout << "\n=> Socket server has been created..." << endl;

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    server_addr.sin_port = htons(portNum);

    if ((bind(client, (struct sockaddr*)&server_addr,sizeof(server_addr))) < 0) 
    {
        cout << "=> Error binding connection, the socket has already been established..." << endl;
        return -1;
    }

    size = sizeof(server_addr);
    cout << "=> Looking for clients..." << endl;

    listen(client, 1);

    int clientCount = 1;
    server = accept(client,(struct sockaddr *)&server_addr,&size);

    // first check if it is valid or not
    if (server < 0) 
        cout << "=> Error on accepting..." << endl;
    
  ///////////////////////////////////////////////
    cout << "Connection made" <<endl;
    while(!isExit && server>0)
    {
		int i;
		for(i=0;i<bufsize;i++)
			buffer[i]=' ';
		recv(server, buffer, bufsize, 0);
		
		
		for (hal::LedValue& led : image1d.leds) {
			led.red = 0;
			led.green = 0;
			led.blue = 0;
			led.white = 0;
		}  
		if (buffer[0]=='u' && buffer[1]=='p')
		{ 
			led=~led;
	    }	
	    if(led)
	    { 
			image1d.leds[22].red=100;
			image1d.leds[22].green=0;
		}
		else
		{
			image1d.leds[22].green=100;
			image1d.leds[22].red=0;
		}
	    
		
		for(i=0;i<5;i++)
			cout << buffer[i];
		cout <<endl;
        
        if(*buffer=='#')
        {
				isExit = true;
		}	
	}

		
	cout << "\n\n=> Connection terminated with IP " << inet_ntoa(server_addr.sin_addr);
	close(server);
	cout << "\nGoodbye..." << endl;
	isExit = false;
	exit(1);

	close(client);
	return 0;

/*
  unsigned counter = 0;

  while (1) {
	for (hal::LedValue& led : image1d.leds) {
      led.red = 0;
      led.green = 0;
      led.blue = 0;
      led.white = 0;
    }  
	image1d.leds[34-(counter)].red=60;
	image1d.leds[34-(counter+1)%35].green=60;
	image1d.leds[34-(counter+2)%35].blue=60;
	image1d.leds[34-(counter+3)%35].white=60;
	
    
    everloop.Write(&image1d);
    counter=(counter+1)%35;
    
    usleep(100000);
    
  }

  return 0;*/
}
