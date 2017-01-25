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

#include "machine/Machine.h"
#include "limits.h"

Scheduler::Scheduler() : readyCount(0), preemption(0), resumption(0), partner(this) {
  Thread* idleThread = Thread::create((vaddr)idleStack, minimumStack);
  idleThread->setAffinity(this)->setPriority(idlePriority);
  // use low-level routines, since runtime context might not exist
  idleThread->stackPointer = stackInit(idleThread->stackPointer, &Runtime::getDefaultMemoryContext(), (ptr_t)Runtime::idleLoop, this, nullptr, nullptr);
  readyQueue[idlePriority].push_back(*idleThread);
  readyCount += 1;
}

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
  GENASSERT0(target);
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

void Scheduler::enqueue(Thread& t) {
  GENASSERT1(t.priority < maxPriority, t.priority);
  readyLock.acquire();
  readyQueue[t.priority].push_back(t);
  bool wake = (readyCount == 0);
  readyCount += 1;
  readyLock.release();
  Runtime::debugS("Thread ", FmtHex(&t), " queued on ", FmtHex(this));
  if (wake) Runtime::wakeUp(this);
}

void Scheduler::resume(Thread& t) {
  GENASSERT1(&t != Runtime::getCurrThread(), Runtime::getCurrThread());
  if (t.nextScheduler) t.nextScheduler->enqueue(t);
  else Runtime::getScheduler()->enqueue(t);
}



void Scheduler::preempt()
{               // IRQs disabled, lock count inflated
#if TESTING_NEVER_MIGRATE
  	switchThread(this);
#else /* migration enabled */

	// Scheduler *target =  Runtime::getCurrThread()->getAffinity();
	Scheduler *target = nullptr;
	mword affinityMask = Runtime::getCurrThread()->getAffinityMask();

	if(affinityMask == 0) {
		 /* use Martin's code when no affinity is set via bit mask */
		target = Runtime::getCurrThread()->getAffinity();
	} else {
		/* CPSC457l: Add code here to scan the affinity mask
		* and select the processor with the smallest ready count.
		* Set the scheduler of the selected processor as target
		* switchThread(target) migrates the current thread to
		* specified target's ready queue
		*/
		//Assume at this point that the mask is valid; assume we checked for this earlier?
		//so mask is a value between decimal 1 and 15 inclusive
		int cores[4] = { -1, -1, -1, -1 }; //default all cores to disallowed

		std::vector<int> _cores;

		Scheduler *schedulerWithMinQueue = nullptr;

		for (int i = 0; i < Machine::getProcessorCount(); i += 1) {
			int bitmask = 1 << i;
			int currentBit = affinityMask & bitmask;

			if (currentBit == 1) {
				Scheduler *_scheduler = Machine::getScheduler(i);

				if (schedulerWithMinQueue == nullptr || schedulerWithMinQueue->readyCount < _scheduler->readyCount) {
					schedulerWithMinQueue = _scheduler;
				}
			}
			// _cores.push_back(currentBit == 1 ? 1 : -1);
		}

		target = schedulerWithMinQueue;






		int affinityMaskCopy = affinityMask; //create a copy of the mask that we can change while retaining the original
		if (affinityMaskCopy >= 8) {
			cores [3] = 1;
			affinityMaskCopy = affinityMaskCopy - 8;
		}
		if (affinityMaskCopy >= 4) {
	  		cores [2] = 1;
			affinityMaskCopy = affinityMaskCopy - 4;
		}
		if (affinityMaskCopy >= 2) {
			cores [1] = 1;
			affinityMaskCopy = affinityMaskCopy - 2;
		}
		if (affinityMaskCopy == 1) {
			cores [0] = 1;
		}
		for (int i = 0; i < 3; i = i + 1) {
			if (cores [i] == 1) {
				//find the ready count of the processor and replace the 1 in cores with that value.
				Scheduler *sched = LocalProcessor::getScheduler();//Machine::getScheduler(i);
				int queueSize = sched->readyCount;
				cores [i] = queueSize;
				target = sched;
			}

			//find the minimum NON-NEGATIVE value in the array and the index will be the new target
			int min = INT_MAX;
			int targ = -1;          //target index
			for (int i = 0; i < 3; i = i + 1) {
				if ((cores [i] != -1) and (cores [i] < min)) { // if cores[i] is non-negative, and minimal
					min = cores [i];    //then store new minimum
					targ = i;           //and new minimum core
				}
			}

			switchThread(LocalProcessor::getScheduler());
			//switchThread(Machine::getScheduler(target)); //switch to queue of core with smallest ready queue
		}
	}






#if TESTING_ALWAYS_MIGRATE
  if (!target) target = partner;
#else /* simple load balancing */
  if (!target) target = (partner->readyCount + 2 < readyCount) ? partner : this;
#endif
  switchThread(target);
#endif
}

void Scheduler::suspend(BasicLock& lk) {
  Runtime::FakeLock fl;
  switchThread(nullptr, lk);
}

void Scheduler::suspend(BasicLock& lk1, BasicLock& lk2) {
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
