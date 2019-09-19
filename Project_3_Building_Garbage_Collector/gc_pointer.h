#include <iostream>
#include <list>
#include <typeinfo>
#include <cstdlib>
#include "gc_details.h"
#include "gc_iterator.h"
/*
    Pointer implements a pointer type that uses
    garbage collection to release unused memory.
    A Pointer must only be used to point to memory
    that was dynamically allocated using new.
    When used to refer to an allocated array,
    specify the array size.
*/
template <class T, int size = 0>
class Pointer{
private:
    // refContainer maintains the garbage collection list.
    static std::list<PtrDetails<T> > refContainer;
    // addr points to the allocated memory to which
    // this Pointer pointer currently points.
    T *addr;
    /*  isArray is true if this Pointer points
        to an allocated array. It is false
        otherwise. 
    */
    bool isArray; 
    // true if pointing to array
    // If this Pointer is pointing to an allocated
    // array, then arraySize contains its size.
    unsigned arraySize; // size of the array
    static bool first; // true when first Pointer is created
    // Return an iterator to pointer details in refContainer.
    typename std::list<PtrDetails<T> >::iterator findPtrInfo(T *ptr);
public:
    // Define an iterator type for Pointer<T>.
    typedef Iter<T> GCiterator;
    // Empty constructor
    // NOTE: templates aren't able to have prototypes with default arguments
    // this is why constructor is designed like this:
    Pointer(){
        Pointer(NULL);
    }
    Pointer(T*);
    // Copy constructor.
    Pointer(const Pointer &);
    // Destructor for Pointer.
    ~Pointer();
    // Collect garbage. Returns true if at least
    // one object was freed.
    static bool collect();
    // Overload assignment of pointer to Pointer.
    T *operator=(T *t);
    // Overload assignment of Pointer to Pointer.
    Pointer &operator=(Pointer &rv);
    // Return a reference to the object pointed
    // to by this Pointer.
    T &operator*(){
        return *addr;
    }
    // Return the address being pointed to.
    T *operator->() { return addr; }
    // Return a reference to the object at the
    // index specified by i.
    T &operator[](int i){ return addr[i];}
    // Conversion function to T *.
    operator T *() { return addr; }
    // Return an Iter to the start of the allocated memory.
    Iter<T> begin(){
        int _size;
        if (isArray)
            _size = arraySize;
        else
            _size = 1;
        return Iter<T>(addr, addr, addr + _size);
    }
    // Return an Iter to one past the end of an allocated array.
    Iter<T> end(){
        int _size;
        if (isArray)
            _size = arraySize;
        else
            _size = 1;
        return Iter<T>(addr + _size, addr, addr + _size);
    }
    // Return the size of refContainer for this type of Pointer.
    static int refContainerSize() { return refContainer.size(); }
    // A utility function that displays refContainer.
    static void showlist();
    // Clear refContainer when program exits.
    static void shutdown();
};

// STATIC INITIALIZATION
// Creates storage for the static variables
template <class T, int size>
std::list<PtrDetails<T> > Pointer<T, size>::refContainer;
template <class T, int size>
bool Pointer<T, size>::first = true;

// Constructor for both initialized and uninitialized objects. -> see class interface
template<class T,int size>
Pointer<T,size>::Pointer(T *t){
    // Register shutdown() as an exit function.
    //std::printf("Entering the Pointer constructor, *t= %d\n", *t); // debug

    if (first)
        atexit(shutdown);
    first = false;

    // TODO: Implement Pointer constructor
    // Lab: Smart Pointer Project Lab
    this->addr = t;
    PtrDetails<T> p(this->addr,this->arraySize);
    //PtrDetails<T> *p= new PtrDetails<T>(this->addr,this->arraySize); //  It does not work!
    //p->memPtr = this->addr; // No need for this as it is done in PtrDetails constructor
    if (this->arraySize)  // decide whether it is an array
    {
        this->isArray = true;
    }
    //std::printf("Entering the Pointer constructor, p->refcount= %d\n", p->refcount); // debug
    refContainer.push_back(p);
}

// Copy constructor.
template< class T, int size>
Pointer<T,size>::Pointer(const Pointer &ob){

    // TODO: Implement Pointer constructor
    // Lab: Smart Pointer Project Lab
    typename std::list<PtrDetails<T> >::iterator p;
    std::printf("Entering the Pointer copy constructor"); // debug
    p = findPtrInfo(ob.addr);
    this->addr = ob.addr; // store the address.
    //p->memPtr = this->addr; // already done in the constructor
    //p->refcount++;     // Wrong logic, we do not have to increment refcount here.
    //std::printf("Entering the Pointer copy constructor, *p= %d\n", *p); // debug
    refContainer.push_back(*p);
    if (this->arraySize)  // decide whether it is an array
    {
        this->isArray = true;
        //p->isArray = true;
    }
}

// Destructor for Pointer.
template <class T, int size>
Pointer<T, size>::~Pointer(){
    
    // TODO: Implement Pointer destructor
    // Lab: New and Delete Project Lab

    typename std::list<PtrDetails<T> >::iterator p;
    p = findPtrInfo(this->addr);
    if (p->refcount)
        p->refcount--;
    if(p->refcount <= 0 )
        (void)Pointer<T, size>::collect();
}

// Collect garbage. Returns true if at least
// one object was freed.
template <class T, int size>
bool Pointer<T, size>::collect(){

    // TODO: Implement collect function
    // LAB: New and Delete Project Lab
    // Note: collect() will be called in the destructor

    bool memfreed = false;
    typename std::list<PtrDetails<T> >::iterator p;
    do{
        // Scan refContainer looking for unreferenced pointers.
        for (p = refContainer.begin(); p != refContainer.end(); p++){
            if(p->refcount > 0 ) // check if the current pointer in the container is in use
                continue;
            else // check if the pointer is unreferenced
            {
                refContainer.remove(*p);
                if (p->memPtr != NULL) // Free memory unless Pointer is null
                    {
                    memfreed = true;   
                    if(p->isArray) // check if the currently allocated memoy is array or not
                        {
                        delete[] p->memPtr;
                        std::printf("Deleting []\n");
                        }
                    else
                        {
                        std::printf("Deleteing single memory\n");
                        delete p->memPtr;
                        }
                    }
            }
            break;
        }
    } while (p != refContainer.end());
    return memfreed;
}

// Overload assignment of pointer to Pointer.
template <class T, int size>
T *Pointer<T, size>::operator=(T *t){

    // TODO: Implement operator=
    // LAB: Smart Pointer Project Lab

    //typename std::list<PtrDetails<T> >::iterator p;
    //std::printf("Entering the assignment of pointer to Pointer, *t= %d\n", *t); // debug
    
    //std::printf("Entering the assignment of pointer to Pointer, *(this->addr)= %d\n", *(this->addr)); // debug
    typename std::list<PtrDetails<T> >::iterator p;
    p = findPtrInfo(this->addr);
    //p->memPtr = this->addr;
    if (p->refcount)
        p->refcount--;
    PtrDetails<T> q (t, 0); // Why we set the size to zero
    q.isArray = false;
    //p->refcount--; // increment ref count
    //std::printf("Entering the assignment of pointer to Pointer, *(p->memPtr)= %d\n", *(p->memPtr)); // debug
    refContainer.push_back(q);
    this->addr = t;      // store the address.
    this->isArray = false;
    this->arraySize = 0;
    return this->addr; 
}
// Overload assignment of Pointer to Pointer.
template <class T, int size>
Pointer<T, size> &Pointer<T, size>::operator=(Pointer &rv){

    // TODO: Implement operator=
    // LAB: Smart Pointer Project Lab
    std::printf("Entering assignment Pointer to Pointer\n");
    typename std::list<PtrDetails<T> >::iterator p_curr;
    typename std::list<PtrDetails<T> >::iterator p_new;
    //p_curr = findPtrInfo(rv.addr);
    p_curr = findPtrInfo(this->addr);
    if(p_curr->refcount)
        p_curr->refcount--;
    //p_new = findPtrInfo(this->addr);
    p_new = findPtrInfo(rv.addr);
    p_new->refcount++;
    this->addr = rv.addr;
    //p_new->memPtr = this->addr;
    //refContainer.push_back(*p_new);
    //return *this;
    return rv;
}

// A utility function that displays refContainer.
template <class T, int size>
void Pointer<T, size>::showlist(){
    typename std::list<PtrDetails<T> >::iterator p;
    std::cout << "refContainer<" << typeid(T).name() << ", " << size << ">:\n";
    std::cout << "memPtr refcount value\n ";
    if (refContainer.begin() == refContainer.end())
    {
        std::cout << " Container is empty!\n\n ";
    }
    for (p = refContainer.begin(); p != refContainer.end(); p++)
    {
        std::cout << "[" << (void *)p->memPtr << "]"
             << " " << p->refcount << " ";
        if (p->memPtr)
            std::cout << " " << *p->memPtr;
        else
            std::cout << "---";
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
// Find a pointer in refContainer.
template <class T, int size>
typename std::list<PtrDetails<T> >::iterator
Pointer<T, size>::findPtrInfo(T *ptr){
    typename std::list<PtrDetails<T> >::iterator p;
    // Find ptr in refContainer.
    for (p = refContainer.begin(); p != refContainer.end(); p++)
        if (p->memPtr == ptr)
            return p;
    return p;
}
// Clear refContainer when program exits.
template <class T, int size>
void Pointer<T, size>::shutdown(){
    if (refContainerSize() == 0)
        return; // list is empty
    typename std::list<PtrDetails<T> >::iterator p;
    for (p = refContainer.begin(); p != refContainer.end(); p++)
    {
        // Set all reference counts to zero
        p->refcount = 0;
    }
    collect();
}