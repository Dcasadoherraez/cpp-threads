
# Multithreading 

## Using functions

Create a new thread: std::thread:
- First parameter: Name of the my_function
- Other parameters: Arguments for the function
- If i need to pass by reference: std::ref(arg1, arg2 ...)
- There is no return value, I need to pass by reference
- Thread starts when created
- Use join() to wait for thread to finish
std::thread t1(my_function, std::ref(arg1, arg2 ...))

## Using functors
Use instances of a class to perform certain operations instead of calling the same function with different inputs.
- First parameter: 
    - Functor object 
    - Reference to functor object (if I need to use member variables later)
- Rest of parameters are passed to the operator() function
- Result can be stored in member variable

```
MyFunctorClass *functor = new MyFunctorClass();
std::thread(std::ref(*functor), arg1, arg2 ...)

class AccumulateFunctor {
 public:
  void operator()(uint64_t start, uint64_t end) {
    _sum = 0;
    for (auto i = start; i < end; i++) {
      _sum += i;
    }
    std::cout << _sum << std::endl;
  }
  // ~AccumulateFunctor(){std::cout << "AccumulateFunctor Destructor." << std::endl;}
  uint64_t _sum;
};
```
## Using lambda functions
[capture](parameters) -> return_type { function_body }
std::vector<int> list{1, 2, 3, 4}
int total = 0;
std::for_each(begin(list), end(list), [&total](int x) { total += x; });
Easy way to perform computations in little space

```
std::thread([i, &partial_sums, step] {
      for (uint64_t j = i * step; j < (i + 1) * step; j++) {
        partial_sums[i] += j;
      }
})
```

--------- All the previous ones require join after the thread is done ---------
## Using async
If my function will return some value.
- Tasks are created with std::async
- Future: Returned value from std::async. Main thread blocks until the future value becomes ready
- Each task starts as soon as it's created.
thread t = async(GetRangeSum, 0, 100/2);
return_value = t.get();

The return value is of type future = The value will come in the future (after calling t.get())
```
std::vector<std::future<uint64_t>> tasks;

for (uint64_t i = 0; i < number_of_threads; i++) {
tasks.push_back(std::async(GetRangeSum, i * step, (i + 1) * step));
}
```
# Mutex and conditional variables
Race conditions: When various threads want to write into the same value
```
auto t1 = std::thread([]() { g_x = 1; });
auto t2 = std::thread([]() { g_x = 2; });
```

- Non atomic operations can lead to non-deterministic behavior:
    - Increment: Read - Increment - Write 
    - Even each of these steps is non-atomic
- Operations get interleaved: One read operation can be interferred 
- Optimizations from CPU might lead to a different behavior from expected (optimizations become observable)

SHARED DATA is the main cause 

Solutions: 
- Mutexes and locks
- Atomic
- Abstraction: CSP, Actors Model, Map-Reduce

## Mutexes and locks
Protect shared memory between threads. Avoid overlapping. Makes operations atommic

### Direct lock:
The other one will wait.
- One mutex per critical section
- Must be a shared variable
- Lock & unlock should match
- If EXCEPTION --> mutex stays locked!! Deadlock

```
std::mutex m;
unsigned long ct;

void Increment() {
    for (int i = 0; i < 100; i++) {
        m.lock();
        ct++;
        m.unlock();
    }
}
```
### Lock Guard
- To avoid deadlock: (Resource Acquisition is Initialization RAII)
- Exits as soon as it goes out of scope (finishes, excption ... ).
- Preferred over simple mutex.

```
void Increment() {
    for (int i = 0; i < 100; i++) {
        std::lock_guard<std::mutex> guard(m);
        ct++;
    }
}
```
### Unique Lock (Lock Guard + Lock/Unlock)
- Automatically locks once constructed
- Unlocks when out of scope
- Optionally lock/unlock

```
void Increment() {
    for (int i = 0; i < 100; i++) {
        std::unique_lock<std::mutex> ul(m);
        ct++;
        ul.unlock();
        // non-critical stuff
        ul.lock();
    }
}
```
### Shared Lock
- Have multiple readers 
- Mutex must be of type std::shared_mutex
- Only a single thread in the writter
- If no writer has locked the shared lock, multiple threads can read at the same time

```
std::shared_mutex m;
unsigned long ct;

// Unique lock for all the writers
void Increment() {
    for (int i = 0; i < 100; i++) {
        std::unique_lock<std::shared_mutex> ul(m);
        ct++;
    }
}


// Shared lock for all the readers
std::shared_lock<std::shared_mutex> sl(m);
std::cout << "Count: " << ct << endl;
```

### Multiple locks
Can cause deadlock if not addressed properly

```
std::mutex m1, m2;
unsigned long ct;

void Increment_bad1() {
    for (int i = 0; i < 100; i++) {
        std::unique_lock<std::mutex> lock1(m1);
        std::unique_lock<std::mutex> lock1(m2);
        ct++;
    }    
}

void Increment_bad2() {
    for (int i = 0; i < 100; i++) {
        std::unique_lock<std::mutex> lock2(m2);
        std::unique_lock<std::mutex> lock2(m1);
        ct++;
    }    
}
```

Solution: Lock all or none --> std::scoped_lock(m1, m2)
- Locks all or none
- Once out of scope, unlocks automatically

## Conditional variables
Shared data between threads.
Producer/Consumer pattern. One thread produces some data, another thread consumes it, and there must be some synchronization between.

### Shared memory:
- Needs critical section.
- Producer puts data into shared memory and sets flag
- Consumer checks for ready flag, consumes it when available (monitor + sample).
- Not efficient as one thread is kept busy by locking and unlocking to check for data

```
bool ready = false;
int data = 0;

void consumer() {
    std::unique_lock<std::mutex> ul(g_mutex);
    while (!ready) {
        ul.unlock();
        ul.lock();
    }
    std::cout << "got data: " << data << std::endl;
    ready = false;
}

/**
 * Produces data and then sets g_ready to true
 */
void producer() {
    std::unique_lock<std::mutex> ul(g_mutex);
    // Produce data
    data = GenRandomValue();
    // Announce that data is produced
    ready = true;
    ul.unlock();
}
```

### Conditional variable:
Notification channel between producer and consumer. There's a conditional variable in the critical section that acts as a notifier.
- Producer: calls m_cv.notify_one() or m_cv.notify_all() to notify one or all threads. 
- Consumer: call to wait until a certain function returns true --> m_cv.wait(lock1, []() { return ready;})

Two way hanshaking between producer and consumer. The steps are:
1. Create critical section variables (mutex, data, ready, condition variable)
2. Producer:
    a. Gets lock
    b. Produces data
    c. Sets ready flag to true
    d. Unlocks 
    e. Notifies consumer
    f. (Optional) waits for response to continue producing
3. Consumer:
    a. Gets lock 
    b. If blocked, unlocks so that producer can take it (l.unlock())
    c. If unblocked, locks so that consumer can sample the data (l.unlock())
    e. Waits until ready & notification has been received 
    f. Gets data coming from the producer
    g. Unlocks the critical section 
    h. (Optional) notifies producer to continue producing
    i. Consumes the data 

Ready variable (predicate --> return ready;) is essential to avoid spurious wake ups by the OS.
Spurious wake ups can cause race conditions by trying to access the same variable from the consumer and the producer.

Notification can be sent after releasing the lock

Deadlock is a side effect of mutex and lock!
Mutex provides sequential consistency: 
- C++ standard requires that unlock() synchronizes with lock(). If a memory location wants to be read by another thread, I must use mutex to ensure this sequential consistency. 

Sequential consistency ensures that the notification will be seen by the other thread.

Summary: Protect conditional variable using a mutex + Always use predicate

Lock() and wait() are blocking actions.

```
std::mutex g_mutex;
std::condition_variable g_cv;
bool g_ready = false;
int g_data = 0;
void ConsumeData(int& data) {}

void Consumer() {
  int data = 0;
  for (int i = 0; i < 100; i++) {
    std::unique_lock<std::mutex> ul(g_mutex);

    // if blocked, ul.unlock() is automatically called.
    // if unblocked, ul.lock() is automatically called.
    g_cv.wait(ul, []() { return g_ready; });
    // Sample data
    data = g_data;
    std::cout << "data: " << data << std::endl;
    g_ready = false;
    ul.unlock();
    g_cv.notify_one();
    ConsumeData(data);
    ul.lock();
  }
}

void Producer() {
  for (int i = 0; i < 100; i++) {
    std::unique_lock<std::mutex> ul(g_mutex);

    // Produce data
    g_data = GenRandomValue();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    g_ready = true;
    ul.unlock();
    g_cv.notify_one();
    ul.lock();
    g_cv.wait(ul, []() { return g_ready == false; });
  }
}
```


