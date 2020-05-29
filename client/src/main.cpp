#include <iostream>
#include "imitator.h"
#include <alerror/alerror.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

using namespace std;

int main(int argc, char* argv[]) {
   char* ip = "127.0.0.1";
   int port = 9559;

   if(argc > 3) {
      cerr << "Wrong number of arguments!" << endl;
      cerr << "Enter the corresponding NAO ip" << endl;
      exit(2);
   }//if

   if (argc == 2)
      sscanf(argv[1], "%d", &port);
   else if (argc == 3) {
      ip = argv[1];
      sscanf(argv[2], "%d", &port);
   }//else

   Imitator* imitator = new Imitator(ip, port);   
   boost::thread imitatorThread(boost::bind(&Imitator::imitate, imitator));
   system("pause");
   delete imitator;
   return 0;
}
