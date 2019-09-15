// This class defines an element that is stored
// in the garbage collection information list.
//
template <class T>
class PtrDetails
{
  public:
    unsigned refcount = 0; // current reference count
    T *memPtr = nullptr;         // pointer to allocated memory
    /* isArray is true if memPtr points
to an allocated array. It is false
otherwise. */
    bool isArray = false; // true if pointing to array
    /* If memPtr is pointing to an allocated
array, then arraySize contains its size */
    unsigned arraySize = 0; // size of array
    // Here, mPtr points to the allocated memory.
    // If this is an array, then size specifies
    // the size of the array.

    PtrDetails(T * ptr, unsigned array_sz)
    {
        // TODO: Implement PtrDetails
        arraySize = array_sz;
        refcount = 1;
        if (array_sz)
        {
            std::printf("Entering PtrDetails, array_sz = %d\n", array_sz); // debug
            isArray = true;
            //ptr = new T[array_sz];
            memPtr = ptr;
        }
        else 
            {
            //ptr = new T;
            memPtr = ptr;
            std::printf("Entering PtrDetails, *memPtr = %d\n", *memPtr); // debug

            }
        

    }
};
// Overloading operator== allows two class objects to be compared.
// This is needed by the STL list class.
template <class T>
bool operator==(const PtrDetails<T> &ob1,
                const PtrDetails<T> &ob2)
{
    return (ob1.memPtr == ob2.memPtr); // we check of the two ptrDetails objects point to the same memory location or not.
}