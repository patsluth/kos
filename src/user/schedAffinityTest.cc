#include <iostream>
#include "syscalls.h"
#include "pthread.h"

using namespace std;

#define MASK1 0x1  // 00001
#define MASK2 0x3  // 00011
#define MASK3 0x4  // 00100
#define MASK4 0x8  // 01000
#define MASK5 0x6  // 00110
#define MASK6 0x10 // 10000

#define REPETITION 5


void* dummy (void *args) {
   //pinning thread to core 1
   cpu_set_t mask = MASK2;
   sched_setaffinity( 0, sizeof(cpu_set_t), &mask );
   int count = REPETITION; 
   while (count--){
		for( int i=0; i<1000000000; i++) asm("");
	}
}

void whereAmI(){
   /* test on which core the process is running */  
   int count = REPETITION; 
   while (count--){
	   for( int i=0; i<100000000; i++) asm("");
         cout << "I am running on core "<< getcid() << endl; 
   }
}

void printAffinity( cpu_set_t affinityMask ){
   mword cpuCount = get_core_count();
	mword bitmask  = 0x1;
   for( mword i=0; i<cpuCount; i++ ){
		if( (affinityMask & (bitmask << i)) != 0 ){
			cout << "Current process has affinity to core "<<i << endl;
		}
	}
}

int main() {

   int err=5;
   cout << "Test process is running on core "<< getcid() << endl; 
 
   /* sched_setaffinity */
   cout << endl << "SCHED_SETAFFINITY TEST 1" << endl;
   cpu_set_t affinityMask = MASK6;
   //cout << "Setting Mask to "<< affinityMask << " for pid 0 "<< endl; 
   err = sched_setaffinity( 0, sizeof(cpu_set_t), &affinityMask );
   if( err == -1 ){
      cout << "sched_setaffinity unsuccessful" << endl; 
   } else {
      //cout << "sched_setaffinity successful" << endl; 
	   whereAmI();
   }

   cout << endl << "SCHED_SETAFFINITY TEST 2" << endl;
   err = 5;
   affinityMask = MASK1;
   //cout << "Setting Mask to "<< affinityMask << " for pid 2" << endl; 
   err = sched_setaffinity( 2, sizeof(cpu_set_t), &affinityMask );
   if( err == -1 ){
      cout << "sched_setaffinity unsuccessful"<< endl; 
   }else {
      //cout << "sched_setaffinity successful" << endl; 
   	whereAmI();
   }

   cout << endl << "SCHED_SETAFFINITY TEST 3" << endl;
   err = 5;
   affinityMask = MASK1;
   //cout << "Setting Mask to "<< affinityMask << " for pid 0 " << endl; 
   err = sched_setaffinity( 0, sizeof(cpu_set_t), &affinityMask );
   if( err == -1 ){
      cout << "sched_setaffinity unsuccessful" << endl; 
   }else{
      //cout << "sched_setaffinity successful " << endl; 
   	whereAmI();
	}


   cout << endl << "SCHED_SETAFFINITY TEST 4" << endl;
   err = 5;
   affinityMask = MASK5;
   //cout << "Setting Mask to "<< affinityMask << " for pid 0 " << endl; 
   err = sched_setaffinity( 0, sizeof(cpu_set_t), &affinityMask );
   if( err == -1 ){
      cout << "sched_setaffinity unsuccessful" << endl; 
   }else{
      //cout << "sched_setaffinity successful" << endl; 
   	whereAmI();
	}
   pthread_t t1;
   pthread_create(&t1, nullptr, dummy, nullptr);
   whereAmI();
   whereAmI();
   whereAmI();

   /* sched_getaffinity */
   cout << endl << "SCHED_GETAFFINITY TEST 1" << endl;
   err = 5;
   err = sched_getaffinity( 2, sizeof(cpu_set_t), &affinityMask );
   if( err == -1 ){
      cout << "sched_getaffinity unsuccessful" << endl; 
   }else{
  		printAffinity(affinityMask);
   }

    
   cout << endl << "SCHED_GETAFFINITY TEST 2" << endl;
   err = 5;
   err = sched_getaffinity( 0, sizeof(cpu_set_t), &affinityMask );
   if( err == -1 ){
      cout << "sched_getaffinity unsuccessful" << endl; 
   }else{
  		printAffinity(affinityMask);
   }
#if 0
#endif
}
