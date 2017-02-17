/******************************************************************************
    Copyright � 2012-2015 Martin Karsten

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/
#include "runtime/RuntimeImpl.h"
#include "runtime/Scheduler.h"
#include "runtime/Stack.h"
#include "runtime/Thread.h"
#include "kernel/Output.h"

#include "kernel/Tree.h"
#include "kernel/Clock.h"
#include "machine/Machine.h"

Scheduler::Scheduler() : readyCount(0), preemption(0), resumption(0), partner(this)
{
  	Thread* idleThread = Thread::create((vaddr)idleStack, minimumStack);
  	idleThread->setAffinity(this)->setPriority(idlePriority);
  	// use low-level routines, since runtime context might not exist
  	idleThread->stackPointer = stackInit(idleThread->stackPointer, &Runtime::getDefaultMemoryContext(), (ptr_t)Runtime::idleLoop, this, nullptr, nullptr);
  	readyQueue[idlePriority].push_back(*idleThread);
  	readyCount += 1;
}

static Tree<ThreadNode> *_globalThreadTree;	// Don't access directly!!
static inline Tree<ThreadNode> *globalThreadTree()
{
	if (_globalThreadTree == NULL) {
		_globalThreadTree = new Tree<ThreadNode>();
	}
	return _globalThreadTree;
}

// Initialize static variables
uint64_t Scheduler::minGranularity = 1;
uint64_t Scheduler::epochLength = 1;
uint64_t Scheduler::defaultEpochLength = 1;

static inline void unlock() {}

template<typename... Args>
static inline void unlock(BasicLock &l, Args&... a) {
  l.release();
  unlock(a...);
}

// very simple N-class prio scheduling!
template<typename... Args>
inline void Scheduler::switchThread(Scheduler* target, Args&... a) {
  preemption += 1;
  CHECK_LOCK_MIN(sizeof...(Args));
  Thread* nextThread;
  readyLock.acquire();
//  for (mword i = 0; i < (target ? idlePriority : maxPriority); i += 1) {
  for (mword i = 0; i < ((target == this) ? idlePriority : maxPriority);
i += 1) {
    if (!readyQueue[i].empty()) {
      nextThread = readyQueue[i].pop_front();
      readyCount -= 1;
      goto threadFound;
    }
  }
  readyLock.release();

	if (target == nullptr) {
		return;
	}

  GENASSERT0(!sizeof...(Args));
  return;                                         // return to current thread

threadFound:







  readyLock.release();
  resumption += 1;
  Thread* currThread = Runtime::getCurrThread();
  GENASSERTN(currThread && nextThread && nextThread != currThread, currThread, ' ', nextThread);

  if (target) currThread->nextScheduler = target; // yield/preempt to given processor
  else currThread->nextScheduler = this;          // suspend/resume to same processor
  unlock(a...);                                   // ...thus can unlock now
  CHECK_LOCK_COUNT(1);
  Runtime::debugS("Thread switch <", (target ? 'Y' : 'S'), ">: ", FmtHex(currThread), '(', FmtHex(currThread->stackPointer), ") to ", FmtHex(nextThread), '(', FmtHex(nextThread->stackPointer), ')');

  Runtime::MemoryContext& ctx = Runtime::getMemoryContext();
  Runtime::setCurrThread(nextThread);


	nextThread->startTime = Machine::getRTC().tick();









  Thread* prevThread = stackSwitch(currThread, target, &currThread->stackPointer, nextThread->stackPointer);
  // REMEMBER: Thread might have migrated from other processor, so 'this'
  //           might not be currThread's Scheduler object anymore.
  //           However, 'this' points to prevThread's Scheduler object.
  Runtime::postResume(false, *prevThread, ctx);
  if (currThread->state == Thread::Cancelled) {
    currThread->state = Thread::Finishing;
    switchThread(nullptr);
    unreachable();
  }
}

extern "C" Thread* postSwitch(Thread* prevThread, Scheduler* target) {
  CHECK_LOCK_COUNT(1);
  if fastpath(target) Scheduler::resume(*prevThread);
  return prevThread;
}

extern "C" void invokeThread(Thread* prevThread, Runtime::MemoryContext* ctx, funcvoid3_t func, ptr_t arg1, ptr_t arg2, ptr_t arg3) {
  Runtime::postResume(true, *prevThread, *ctx);
  func(arg1, arg2, arg3);
  Runtime::getScheduler()->terminate();
}

void Scheduler::enqueue(Thread& t)
{
  	GENASSERT1(t.priority < maxPriority, t.priority);
  	readyLock.acquire();
  	readyQueue[t.priority].push_back(t);
  	bool wake = (readyCount == 0);
  	readyCount += 1;
  	readyLock.release();
  	Runtime::debugS("Thread ", FmtHex(&t), " queued on ", FmtHex(this));
  	if (wake) Runtime::wakeUp(this);






	int numTasks = (globalThreadTree()->empty() == false) ? globalThreadTree()->root->size : 0;

	if (Scheduler::defaultEpochLength <= (Scheduler::minGranularity * numTasks)) {
		Scheduler::epochLength = Scheduler::defaultEpochLength;
	} else {
		Scheduler::epochLength = (numTasks * Scheduler::minGranularity);
	}

	if (globalThreadTree()->empty() == false) {
		ThreadNode *minThreadNode = globalThreadTree()->readMinNode();
		t.vRuntime = minThreadNode->thread->vRuntime;
	} else {
		t.vRuntime = 0;
	}
}

void Scheduler::resume(Thread& t)
{
	GENASSERT1(&t != Runtime::getCurrThread(), Runtime::getCurrThread());

	if (globalThreadTree()->empty() == false) {
		ThreadNode *minThreadNode = globalThreadTree()->readMinNode();
		t.vRuntime -= minThreadNode->thread->vRuntime;
		ThreadNode *threadNode = new ThreadNode(&t);
		globalThreadTree()->insert(*threadNode);	// insert currentThread back into tree
	}

  	if (t.nextScheduler) t.nextScheduler->enqueue(t);
  	else Runtime::getScheduler()->enqueue(t);
}



void Scheduler::preempt() {               // IRQs disabled, lock count inflated
#if TESTING_NEVER_MIGRATE
  switchThread(this);
#else /* migration enabled */
  //Scheduler* target =  Runtime::getCurrThread()->getAffinity();
  Scheduler *target = nullptr;
  mword affinityMask = Runtime::getCurrThread()->getAffinityMask();

  if( affinityMask == 0 ) {
	  /* use Martin's code when no affinity is set via bit mask */
	  target =  Runtime::getCurrThread()->getAffinity();
   }  else {
	  /* CPSC457l: Add code here to scan the affinity mask
      * and select the processor with the smallest ready count.
      * Set the scheduler of the selected processor as target
      * switchThread(target) migrates the current thread to
      * specified target's ready queue
      */

   }





	Thread* currentThread = Runtime::getCurrThread();
	Thread* nextThread = NULL;






   if (currentThread != nullptr) { // && nextThread != nullptr && nextThread->nextScheduler == this) {

		uint64_t threadRuntime = Machine::getRTC().tick() - currentThread->startTime;
		currentThread->vRuntime += (threadRuntime * currentThread->priority);

		//KOUT::outl("cthreadRuntime: ", threadRuntime);
		//KOUT::outl("currentThread->vRuntime: ", currentThread->vRuntime);

		// psuedocode
		// update vRuntime of currentThread
		// If currentThread has already run for minGranularity
			// If currentThread.vRuntime < leftmostThread.vRuntime
				// continue currentThread
			// Else
				// insert currThread back into tree
				// Pick leftmostThread to run



		if (currentThread->vRuntime >= Scheduler::minGranularity) {
			if (globalThreadTree()->empty() == false && globalThreadTree()->readMinNode() != NULL) {
				ThreadNode *minThreadNode = globalThreadTree()->popMinNode();
				Thread *minThread = minThreadNode->thread;

				if (currentThread->vRuntime < minThread->vRuntime) {
					// continue currThread
				} else {
					ThreadNode *currentThreadNode = new ThreadNode(currentThread);
					globalThreadTree()->insert(*currentThreadNode);	// insert currentThread back into tree
					nextThread = minThread; 						// pick leftmostThread to run
				}
			}
		}
	}


	//target = nextThread->nextScheduler;


   	//auto foundNode = Scheduler::globalProcessTree->find(*currThreadNode);
   //  	ThreadNode _ttt = (ThreadNode)foundNode->item;
   //KOUT::outl(_ttt.th->priority);

   //if (*&(_ttt.th->nextScheduler) == this) {
   //} else {
    // threadTree->insert(*findNode);
   	//if (foundNode != nullptr) {
   	 //auto __test = (ThreadNode)(foundNode->item);
   	 // if (__test.th->nextScheduler == this) {
   	// if (((ThreadNode)(foundNode->item)).th == findNode->th) {
   			//KOUT::outl("A");
   			//this->threadTree->erase(foundNode);
   // } else {
   	//   /	KOUT::outl("B");
   // }
   //}
   // else {


   		//ThreadNode *nextThreadNode = new ThreadNode(nextThread);
   	// if (((ThreadNode)(foundNode->item)).th == findNode->th) {
   		//  	KOUT::outl(currThread->nextScheduler->threadTree->root->size);
   		//this->threadTree->insert(*nextThreadNode);
   	// threadTree->insert(*findNode);
   		//KOUT::outl("C");
   // }



#if TESTING_ALWAYS_MIGRATE
  if (!target) target = partner;
#else /* simple load balancing */
  if (!target) target = (partner->readyCount + 2 < readyCount) ? partner : this;
#endif

  switchThread(target);
#endif
}

void Scheduler::suspend(BasicLock& lk)
{
	Thread *currentThread = Runtime::getCurrThread();

 	if (currentThread != nullptr) {
	 	if (globalThreadTree()->empty() == false) {
		 	ThreadNode *minThreadNode = globalThreadTree()->readMinNode();
		 	currentThread->vRuntime -= minThreadNode->thread->vRuntime;
	 	}
 	}

	Runtime::FakeLock fl;
	switchThread(nullptr, lk);
}

void Scheduler::suspend(BasicLock& lk1, BasicLock& lk2)
{
	Thread *currentThread = Runtime::getCurrThread();

 	if (currentThread != nullptr) {
	 	if (globalThreadTree()->empty() == false) {
		 	ThreadNode *minThreadNode = globalThreadTree()->readMinNode();
		 	currentThread->vRuntime -= minThreadNode->thread->vRuntime;
	 	}
 	}

  	Runtime::FakeLock fl;
  	switchThread(nullptr, lk1, lk2);
}

void Scheduler::terminate() {
  Runtime::RealLock rl;
  Thread* thr = Runtime::getCurrThread();
  GENASSERT1(thr->state != Thread::Blocked, thr->state);
  thr->state = Thread::Finishing;
  switchThread(nullptr);
  unreachable();
}

void Scheduler::yield(){
  Runtime::RealLock rl;
  preempt();
}
